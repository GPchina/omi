#include "transport.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/ring_buffer.h>

#include "config.h"
#include "utils.h"
// #include "nfc.h"
#include "button.h"
#include "lib/battery/battery.h"
#include "mic.h"
#include "sdcard.h"
#include "speaker.h"
#include "storage.h"
// #include "friend.h"
LOG_MODULE_REGISTER(transport, CONFIG_LOG_DEFAULT_LEVEL);

#define MAX_STORAGE_BYTES 0xFFFF0000
#define MAX_AUDIO_FILES 10
extern bool is_connected;
extern bool storage_is_on;
extern uint8_t file_count;
// P14: file_num_array[0..9] = a01..a10 文件大小, [10] = 预留 offset
extern uint32_t file_num_array[MAX_AUDIO_FILES + 1];
struct bt_conn *current_connection = NULL;
uint16_t current_mtu = 0;
uint16_t current_package_index = 0;

//
// P15: unified recording state machine (manual button / auto VAD).
//
// Two recording modes, both persisting to the SD card regardless of BLE state:
//   - REC_MANUAL: button-toggled (single tap start, single tap stop). VAD is
//     completely ignored in this mode - it follows the button on/off rule.
//   - REC_AUTO: voice-activated (VAD). Starts on first detected speech, ends
//     after VAD_SILENCE_TIMEOUT_MS of continuous silence.
//
// The pusher writes to the SD card whenever a recording is active (manual or
// auto), independent of BLE connection/subscription. Live BLE streaming only
// happens when NO recording is active AND a central is subscribed.
//
// These are shared between the PDM IRQ context (record_feed_pcm), the button
// work-queue thread (record_button_toggle) and the pusher thread - volatile so
// the compiler re-reads them every time (single-core nRF52, 32-bit aligned
// accesses are atomic; the bool is single-byte).
static volatile rec_mode_t rec_mode = REC_IDLE;
static volatile uint32_t rec_silence_ms = 0;    // auto-recording continuous-silence timer
static volatile uint32_t rec_file_bytes = 0;    // bytes written to the current file (truncation)
// Deferred file allocation. record_feed_pcm() runs in the PDM IRQ context and
// must NOT do filesystem IO (start_new_recording does fs_stat/fs_open), so it
// only marks state here and the pusher thread performs the actual file switch.
static volatile bool rec_needs_new_file = false;

// VAD: compare sum of squares against threshold^2 * count to avoid sqrt/float.
// Threshold is RMS of the 16-bit PCM; 400 is a conservative speech level and
// should be re-calibrated against the real mic once on hardware.
#define VAD_RMS_THRESHOLD 400
// Auto recording ends after 10s of continuous silence.
#define VAD_SILENCE_TIMEOUT_MS 10000
// Single-file truncation: ~30s @32kbps Opus (~4KB/s). Keeps each recording
// short enough for fast medium-model transcription and BLE download.
#define MAX_AUDIO_FILE_SIZE 120000

extern int start_new_recording(void);   // sdcard.c (not in devkit sdcard.h)

// Mark recording active + request a fresh file. Safe in any context: it only
// flips flags; the actual start_new_recording() runs in the pusher thread.
static void rec_mark_start(rec_mode_t mode)
{
    rec_mode = mode;
    rec_silence_ms = 0;
    rec_file_bytes = 0;
    rec_needs_new_file = true;
}

static void rec_stop(void)
{
    rec_mode = REC_IDLE;
    rec_silence_ms = 0;
}

bool is_recording(void)
{
    return rec_mode != REC_IDLE;
}

rec_mode_t get_rec_mode(void)
{
    return rec_mode;
}

void record_button_toggle(void)
{
    if (rec_mode == REC_MANUAL) {
        rec_stop();                    // manual -> stop
    } else {
        rec_mark_start(REC_MANUAL);    // idle or auto -> (re)start manual
    }
}

void record_feed_pcm(const int16_t *samples, size_t count)
{
    // Integer sum-of-squares energy; compare to threshold^2 * count (no sqrt).
    int64_t sumsq = 0;
    for (size_t i = 0; i < count; i++) {
        int32_t s = samples[i];
        sumsq += (int64_t)s * s;
    }
    bool voice = sumsq > (int64_t)VAD_RMS_THRESHOLD * VAD_RMS_THRESHOLD * (int64_t)count;

    if (rec_mode == REC_MANUAL) {
        return;                        // manual mode: VAD has no say
    }

    if (rec_mode == REC_IDLE) {
        if (voice) {
            rec_mark_start(REC_AUTO);  // first speech -> auto recording
        }
    } else if (rec_mode == REC_AUTO) {
        if (voice) {
            rec_silence_ms = 0;
        } else {
            rec_silence_ms += 100;     // mic_handler cadence = 100ms
            if (rec_silence_ms >= VAD_SILENCE_TIMEOUT_MS) {
                rec_stop();
            }
        }
    }
}

bool should_capture_audio(void)
{
    // Feed the transport pipeline only when the audio has somewhere to go:
    // a BLE central (live stream) or an active recording (manual or auto).
    return is_connected || is_recording();
}

//
// Internal
//

struct k_mutex write_sdcard_mutex;

static ssize_t audio_data_write_handler(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf,
                                        uint16_t len,
                                        uint16_t offset,
                                        uint8_t flags);

static struct bt_conn_cb _callback_references;
static struct bt_gatt_cb _gatt_callbacks;
static void audio_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t audio_data_read_characteristic(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              void *buf,
                                              uint16_t len,
                                              uint16_t offset);
static ssize_t audio_codec_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset);

static void dfu_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t dfu_control_point_write_handler(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf,
                                               uint16_t len,
                                               uint16_t offset,
                                               uint8_t flags);

//
// Service and Characteristic
//
// Audio service with UUID 19B10000-E8F2-537E-4F6C-D104768A1214
// exposes following characteristics:
// - Audio data (UUID 19B10001-E8F2-537E-4F6C-D104768A1214) to send audio data (read/notify)
// - Audio codec (UUID 19B10002-E8F2-537E-4F6C-D104768A1214) to send audio codec type (read)
// TODO: The current audio service UUID seems to come from old Intel sample code,
// we should change it to UUID 814b9b7c-25fd-4acd-8604-d28877beee6d
static struct bt_uuid_128 audio_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10000, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_data_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10001, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_format_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10002, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));
static struct bt_uuid_128 audio_characteristic_speaker_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x19B10003, 0xE8F2, 0x537E, 0x4F6C, 0xD104768A1214));

static struct bt_gatt_attr audio_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&audio_service_uuid),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_data_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           audio_data_read_characteristic,
                           NULL,
                           NULL),
    BT_GATT_CCC(audio_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&audio_characteristic_format_uuid.uuid,
                           BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ,
                           audio_codec_read_characteristic,
                           NULL,
                           NULL),
#ifdef CONFIG_OMI_ENABLE_SPEAKER
    BT_GATT_CHARACTERISTIC(&audio_characteristic_speaker_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE,
                           NULL,
                           audio_data_write_handler,
                           NULL),
    BT_GATT_CCC(audio_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), //
#endif

};

static struct bt_gatt_service audio_service = BT_GATT_SERVICE(audio_service_attr);

// Nordic Legacy DFU service with UUID 00001530-1212-EFDE-1523-785FEABCD123
// exposes following characteristics:
// - Control point (UUID 00001531-1212-EFDE-1523-785FEABCD123) to start the OTA update process (write/notify)
static struct bt_uuid_128 dfu_service_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00001530, 0x1212, 0xEFDE, 0x1523, 0x785FEABCD123));
static struct bt_uuid_128 dfu_control_point_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x00001531, 0x1212, 0xEFDE, 0x1523, 0x785FEABCD123));

static struct bt_gatt_attr dfu_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&dfu_service_uuid),
    BT_GATT_CHARACTERISTIC(&dfu_control_point_uuid.uuid,
                           BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_WRITE,
                           NULL,
                           dfu_control_point_write_handler,
                           NULL),
    BT_GATT_CCC(dfu_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
};

static struct bt_gatt_service dfu_service = BT_GATT_SERVICE(dfu_service_attr);
// Acceleration data
// this code activates the onboard accelerometer. some cute ideas may include shaking the necklace to color strobe
//
static struct sensors mega_sensor;
static struct device *lsm6dsl_dev;
// Arbritrary uuid, feel free to change
static struct bt_uuid_128 accel_uuid =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x32403790, 0x0000, 0x1000, 0x7450, 0xBF445E5829A2));
static struct bt_uuid_128 accel_uuid_x =
    BT_UUID_INIT_128(BT_UUID_128_ENCODE(0x32403791, 0x0000, 0x1000, 0x7450, 0xBF445E5829A2));

static void accel_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value);
static ssize_t accel_data_read_characteristic(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              void *buf,
                                              uint16_t len,
                                              uint16_t offset);

static struct bt_gatt_attr accel_service_attr[] = {
    BT_GATT_PRIMARY_SERVICE(&accel_uuid), // primary description
    BT_GATT_CHARACTERISTIC(&accel_uuid_x.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           accel_data_read_characteristic,
                           NULL,
                           NULL),                                                          // data type
    BT_GATT_CCC(accel_ccc_config_changed_handler, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE), // scheduler
};
static struct bt_gatt_service accel_service = BT_GATT_SERVICE(accel_service_attr);

static ssize_t accel_data_read_characteristic(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              void *buf,
                                              uint16_t len,
                                              uint16_t offset)
{
    LOG_INF("Acceleration data read characteristic");
    int axis_mode = 6; // 3 for accel, 6 for (also) gyro
    return bt_gatt_attr_read(conn, attr, buf, len, offset, &axis_mode, sizeof(axis_mode));
}

#define ACCEL_REFRESH_INTERVAL 1000 // 1.0 seconds

void broadcast_accel(struct k_work *work_item);
K_WORK_DELAYABLE_DEFINE(accel_work, broadcast_accel);

void broadcast_accel(struct k_work *work_item)
{

    sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ);
    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_X, &mega_sensor.a_x);
    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Y, &mega_sensor.a_y);
    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Z, &mega_sensor.a_z);

    sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ);
    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &mega_sensor.g_x);
    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &mega_sensor.g_y);
    sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Z, &mega_sensor.g_z);

    // only time mega sensor is changed is through here (hopefully),  so no chance of race condition
    int err = bt_gatt_notify(current_connection, &accel_service.attrs[1], &mega_sensor, sizeof(mega_sensor));
    if (err) {
        LOG_ERR("Error updating Accelerometer data");
    }
    k_work_reschedule(&accel_work, K_MSEC(ACCEL_REFRESH_INTERVAL));
}

struct gpio_dt_spec accel_gpio_pin = {.port = DEVICE_DT_GET(DT_NODELABEL(gpio1)),
                                      .pin = 8,
                                      .dt_flags = GPIO_INT_DISABLE};

// use d4,d5
static void accel_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Client subscribed for notifications");
    } else if (value == 0) {
        LOG_INF("Client unsubscribed from notifications");
    } else {
        LOG_ERR("Invalid CCC value: %u", value);
    }
}

int accel_start()
{
    struct sensor_value odr_attr;
    lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    k_msleep(50);
    if (lsm6dsl_dev == NULL) {
        LOG_ERR("Could not get LSM6DSL device");
        return 0;
    }
    if (!device_is_ready(lsm6dsl_dev)) {
        LOG_ERR("LSM6DSL: not ready");
        return 0;
    }
    odr_attr.val1 = 10;
    odr_attr.val2 = 0;

    if (gpio_is_ready_dt(&accel_gpio_pin)) {
        LOG_PRINTK("Speaker Pin ready\n");
    } else {
        LOG_PRINTK("Error setting up speaker Pin\n");
        return -1;
    }
    if (gpio_pin_configure_dt(&accel_gpio_pin, GPIO_OUTPUT_INACTIVE) < 0) {
        LOG_PRINTK("Error setting up Haptic Pin\n");
        return -1;
    }
    gpio_pin_set_dt(&accel_gpio_pin, 1);
    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("Cannot set sampling frequency for Accelerometer.");
        return 0;
    }
    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        LOG_ERR("Cannot set sampling frequency for gyro.");
        return 0;
    }
    if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
        LOG_ERR("Sensor sample update error");
        return 0;
    }

    LOG_INF("Accelerometer is ready for use \n");

    return 1;
}
// Advertisement data
static const struct bt_data bt_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_UUID128_ALL, audio_service_uuid.val, sizeof(audio_service_uuid.val)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

// Scan response data
static const struct bt_data bt_sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
    BT_DATA(BT_DATA_UUID128_ALL, dfu_service_uuid.val, sizeof(dfu_service_uuid.val)),
};

//
// State and Characteristics
//

static void audio_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Client subscribed for notifications");
    } else if (value == 0) {
        LOG_INF("Client unsubscribed from notifications");
    } else {
        LOG_INF("Invalid CCC value: %u", value);
    }
}

static ssize_t audio_data_read_characteristic(struct bt_conn *conn,
                                              const struct bt_gatt_attr *attr,
                                              void *buf,
                                              uint16_t len,
                                              uint16_t offset)
{
    LOG_DBG("audio_data_read_characteristic");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, NULL, 0);
}

static ssize_t audio_codec_read_characteristic(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               void *buf,
                                               uint16_t len,
                                               uint16_t offset)
{
    uint8_t value[1] = {CODEC_ID};
    LOG_DBG("audio_codec_read_characteristic %d", CODEC_ID);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(value));
}

static ssize_t audio_data_write_handler(struct bt_conn *conn,
                                        const struct bt_gatt_attr *attr,
                                        const void *buf,
                                        uint16_t len,
                                        uint16_t offset,
                                        uint8_t flags)
{
    uint16_t amount = 400;
    int16_t *int16_buf = (int16_t *) buf;
    uint8_t *data = (uint8_t *) buf;
    bt_gatt_notify(conn, attr, &amount, sizeof(amount));
    amount = speak(len, buf);
    return len;
}

//
// DFU Service Handlers
//

static void dfu_ccc_config_changed_handler(const struct bt_gatt_attr *attr, uint16_t value)
{
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Client subscribed for notifications");
    } else if (value == 0) {
        LOG_INF("Client unsubscribed from notifications");
    } else {
        LOG_INF("Invalid CCC value: %u", value);
    }
}

static ssize_t dfu_control_point_write_handler(struct bt_conn *conn,
                                               const struct bt_gatt_attr *attr,
                                               const void *buf,
                                               uint16_t len,
                                               uint16_t offset,
                                               uint8_t flags)
{
    LOG_INF("dfu_control_point_write_handler");
    if (len == 1 && ((uint8_t *) buf)[0] == 0x06) {
        NRF_POWER->GPREGRET = 0xA8;
        NVIC_SystemReset();
    } else if (len == 2 && ((uint8_t *) buf)[0] == 0x01) {
        uint8_t notification_value = 0x10;
        bt_gatt_notify(conn, attr, &notification_value, sizeof(notification_value));

        NRF_POWER->GPREGRET = 0xA8;
        NVIC_SystemReset();
    }
    return len;
}

//
// Battery Service Handlers
//

#define BATTERY_REFRESH_INTERVAL 15000 // 15 seconds

void broadcast_battery_level(struct k_work *work_item);

K_WORK_DELAYABLE_DEFINE(battery_work, broadcast_battery_level);

void broadcast_battery_level(struct k_work *work_item)
{
    uint16_t battery_millivolt;
    uint8_t battery_percentage;
    if (battery_get_millivolt(&battery_millivolt) == 0 &&
        battery_get_percentage(&battery_percentage, battery_millivolt) == 0) {

        LOG_PRINTK("Battery at %d mV (capacity %d%%)\n", battery_millivolt, battery_percentage);

        // Use the Zephyr BAS function to set (and notify) the battery level
        int err = bt_bas_set_battery_level(battery_percentage);
        if (err) {
            LOG_ERR("Error updating battery level: %d", err);
        }
    } else {
        LOG_ERR("Failed to read battery level");
    }

    k_work_reschedule(&battery_work, K_MSEC(BATTERY_REFRESH_INTERVAL));
}

//
// Connection Callbacks
//

static void _transport_connected(struct bt_conn *conn, uint8_t err)
{
    struct bt_conn_info info = {0};
    storage_is_on = true;

    err = bt_conn_get_info(conn, &info);
    if (err) {
        LOG_ERR("Failed to get connection info (err %d)", err);
        bt_conn_unref(conn);
        return;
    }

    LOG_INF("bluetooth activated");

    if (current_connection != NULL) {
        bt_conn_unref(current_connection);
    }
    current_connection = bt_conn_ref(conn);
    /* Use the negotiated ATT MTU for the notify path, not the LE data
     * length. The original code used info.le.data_len->tx_max_len as
     * "current_mtu", which is only valid while DLE is enabled (251).
     * With DLE disabled for Windows bring-up, data_len is 27 and the pusher
     * gate (current_mtu < MINIMAL_PACKET_SIZE) would silence audio forever.
     * bt_gatt_get_mtu() returns the real ATT MTU (498 here); it starts at
     * 23 and _att_mtu_updated() raises it once the central's MTU exchange
     * completes. The pusher stays silent until then, which is correct. */
    current_mtu = bt_gatt_get_mtu(conn);
    LOG_INF("Transport connected (att_mtu=%d)", current_mtu);
    LOG_DBG("Interval: %d, latency: %d, timeout: %d", info.le.interval, info.le.latency, info.le.timeout);
#if defined(CONFIG_BT_PHY_UPDATE)
    LOG_DBG("TX PHY %s, RX PHY %s", phy2str(info.le.phy->tx_phy), phy2str(info.le.phy->rx_phy));
#endif
#if defined(CONFIG_BT_DATA_LEN_UPDATE)
    LOG_DBG("LE data len updated: TX (len: %d time: %d) RX (len: %d time: %d)",
            info.le.data_len->tx_max_len,
            info.le.data_len->tx_max_time,
            info.le.data_len->rx_max_len,
            info.le.data_len->rx_max_time);
#endif

    k_work_schedule(&battery_work, K_MSEC(100)); // run immediately

    is_connected = true;
}

static void _transport_disconnected(struct bt_conn *conn, uint8_t err)
{
    is_connected = false;
    storage_is_on = false;

    LOG_INF("Transport disconnected");

    if (current_connection != NULL) {
        bt_conn_unref(current_connection);
        current_connection = NULL;
    }
    current_mtu = 0;
}

static bool _le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    LOG_INF("Transport connection parameters update request received.");
    LOG_DBG("Minimum interval: %d, Maximum interval: %d", param->interval_min, param->interval_max);
    LOG_DBG("Latency: %d, Timeout: %d", param->latency, param->timeout);

    return true;
}

static void _le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    LOG_INF("Connection parameters updated.");
    LOG_DBG("[ interval: %d, latency: %d, timeout: %d ]", interval, latency, timeout);
}

#if defined(CONFIG_BT_PHY_UPDATE)
static void _le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
    // LOG_DBG("LE PHY updated: TX PHY %s, RX PHY %s",
    //        phy2str(param->tx_phy), phy2str(param->rx_phy));
}
#endif

#if defined(CONFIG_BT_DATA_LEN_UPDATE)
static void _le_data_length_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    LOG_DBG("LE data len updated: TX (len: %d time: %d)"
            " RX (len: %d time: %d)",
            info->tx_max_len,
            info->tx_max_time,
            info->rx_max_len,
            info->rx_max_time);
    current_mtu = info->tx_max_len;
}
#endif

static void _att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
    LOG_INF("ATT MTU updated: tx=%d rx=%d", tx, rx);
    if (conn == current_connection) {
        current_mtu = tx;
    }
}

/* NOTE: att_mtu_updated is a bt_gatt_cb member, NOT a bt_conn_cb member. */
static struct bt_gatt_cb _gatt_callbacks = {
    .att_mtu_updated = _att_mtu_updated,
};

static struct bt_conn_cb _callback_references = {
    .connected = _transport_connected,
    .disconnected = _transport_disconnected,
    .le_param_req = _le_param_req,
    .le_param_updated = _le_param_updated,
#if defined(CONFIG_BT_PHY_UPDATE)
    .le_phy_updated = _le_phy_updated,
#endif
#if defined(CONFIG_BT_DATA_LEN_UPDATE)
    .le_data_len_updated = _le_data_length_updated,
#endif
};

//
// Ring Buffer
//

#define NET_BUFFER_HEADER_SIZE 3
#define RING_BUFFER_HEADER_SIZE 2
static uint8_t tx_queue[NETWORK_RING_BUF_SIZE * (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE)];
static uint8_t tx_buffer[CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE];
static uint8_t tx_buffer_2[CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE];
static uint32_t tx_buffer_size = 0;
static struct ring_buf ring_buf;

static bool write_to_tx_queue(uint8_t *data, size_t size)
{
    if (size > CODEC_OUTPUT_MAX_BYTES) {
        return false;
    }

    // Copy data (TODO: Avoid this copy)
    tx_buffer_2[0] = size & 0xFF;
    tx_buffer_2[1] = (size >> 8) & 0xFF;
    memcpy(tx_buffer_2 + RING_BUFFER_HEADER_SIZE, data, size);

    // Write to ring buffer
    int written =
        ring_buf_put(&ring_buf,
                     tx_buffer_2,
                     (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE)); // It always fits completely or not at all
    if (written != CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE) {
        return false;
    } else {
        return true;
    }
}

static bool read_from_tx_queue()
{

    // Read from ring buffer
    // memset(tx_buffer, 0, sizeof(tx_buffer));
    tx_buffer_size =
        ring_buf_get(&ring_buf,
                     tx_buffer,
                     (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE)); // It always fits completely or not at all
    if (tx_buffer_size != (CODEC_OUTPUT_MAX_BYTES + RING_BUFFER_HEADER_SIZE)) {
        // P8: demoted from LOG_ERR to LOG_DBG. In deferred log mode this is a
        // normal transient condition (pusher spins faster than codec fills the
        // ring buffer) and the old error flooded the log pipe with thousands
        // of messages/sec, hiding real errors. BLE audio path unaffected.
        LOG_DBG("Failed to read from ring buffer. not enough data %d", tx_buffer_size);
        return false;
    }

    // Adjust size
    tx_buffer_size = tx_buffer[0] + (tx_buffer[1] << 8);
    // LOG_PRINTK("tx_buffer_size %d\n",tx_buffer_size);

    return true;
}

//
// Pusher
//

// Thread
K_THREAD_STACK_DEFINE(pusher_stack, 4096);
static struct k_thread pusher_thread;
static uint16_t packet_next_index = 0;
static uint8_t pusher_temp_data[CODEC_OUTPUT_MAX_BYTES + NET_BUFFER_HEADER_SIZE];

static bool push_to_gatt(struct bt_conn *conn)
{
    // Read data from ring buffer
    if (!read_from_tx_queue()) {
        return false;
    }

    // Push each frame
    uint8_t *buffer = tx_buffer + RING_BUFFER_HEADER_SIZE;
    uint32_t offset = 0;
    uint8_t index = 0;
    int retry_count = 0;
    const int max_retries = 3;

    while (offset < tx_buffer_size) {
        // Recombine packet
        uint32_t id = packet_next_index++;
        uint32_t packet_size = MIN(current_mtu - NET_BUFFER_HEADER_SIZE, tx_buffer_size - offset);
        pusher_temp_data[0] = id & 0xFF;
        pusher_temp_data[1] = (id >> 8) & 0xFF;
        pusher_temp_data[2] = index;
        memcpy(pusher_temp_data + NET_BUFFER_HEADER_SIZE, buffer + offset, packet_size);

        offset += packet_size;
        index++;

        retry_count = 0;
        while (retry_count < max_retries) {
            // Try send notification
            int err =
                bt_gatt_notify(conn, &audio_service.attrs[1], pusher_temp_data, packet_size + NET_BUFFER_HEADER_SIZE);

            // Log failure
            if (err) {
                LOG_DBG("bt_gatt_notify failed (err %d)", err);
                LOG_DBG("MTU: %d, packet_size: %d", current_mtu, packet_size + NET_BUFFER_HEADER_SIZE);
                k_sleep(K_MSEC(1));
                retry_count++;
                continue;
            }

            // Try to send more data if possible
            if (err == -EAGAIN || err == -ENOMEM) {
                retry_count++;
                continue;
            }

            // Break if success
            break;
        }

        if (retry_count >= max_retries) {
            LOG_ERR("Failed to send packet after %d retries", max_retries);
            return false;
        }
    }

    return true;
}
#define OPUS_PREFIX_LENGTH 1
#define OPUS_PADDED_LENGTH 80
#define MAX_WRITE_SIZE 440
static uint8_t storage_temp_data[MAX_WRITE_SIZE];
static uint32_t offset = 0;
static uint16_t buffer_offset = 0;
// bool write_to_storage(void)
// {
//     if (!read_from_tx_queue())
//     {
//         return false;
//     }

//     uint8_t *buffer = tx_buffer+2;
//     const uint32_t packet_size = tx_buffer_size;
//     //load into write at 400 bytes at a time. is faster
//     memcpy(storage_temp_data + OPUS_PREFIX_LENGTH + buffer_offset, buffer, packet_size);
//     storage_temp_data[buffer_offset] = (uint8_t)tx_buffer_size;

//     buffer_offset = buffer_offset+OPUS_PADDED_LENGTH;
//     if(buffer_offset >= OPUS_PADDED_LENGTH*5) {
//     uint8_t *write_ptr = (uint8_t*)storage_temp_data;
//     write_to_file(write_ptr,OPUS_PADDED_LENGTH*5);

//     buffer_offset = 0;
//     }

//     return true;
// }
// for improving ble bandwidth
// Returns the number of bytes actually flushed to the SD card this call
// (0 when the frame was only buffered and no file write happened yet).
uint32_t write_to_storage(void)
{ // max possible packing
    if (!read_from_tx_queue()) {
        return 0;
    }

    uint8_t *buffer = tx_buffer + 2;
    uint8_t packet_size = (uint8_t) (tx_buffer_size + OPUS_PREFIX_LENGTH);
    uint32_t written = 0;

    // buffer_offset = buffer_offset+amount_to_fill;
    // check if adding the new packet will cause a overflow
    if (buffer_offset + packet_size > MAX_WRITE_SIZE - 1) {

        storage_temp_data[buffer_offset] = tx_buffer_size;
        uint8_t *write_ptr = storage_temp_data;
        write_to_file(write_ptr, MAX_WRITE_SIZE);
        written = MAX_WRITE_SIZE;

        buffer_offset = packet_size;
        storage_temp_data[0] = tx_buffer_size;
        memcpy(storage_temp_data + 1, buffer, tx_buffer_size);

    } else if (buffer_offset + packet_size == MAX_WRITE_SIZE - 1) { // exact frame needed
        storage_temp_data[buffer_offset] = tx_buffer_size;
        memcpy(storage_temp_data + buffer_offset + 1, buffer, tx_buffer_size);
        buffer_offset = 0;
        uint8_t *write_ptr = (uint8_t *) storage_temp_data;
        write_to_file(write_ptr, MAX_WRITE_SIZE);
        written = MAX_WRITE_SIZE;

    } else {
        storage_temp_data[buffer_offset] = tx_buffer_size;
        memcpy(storage_temp_data + buffer_offset + 1, buffer, tx_buffer_size);
        buffer_offset = buffer_offset + packet_size;
    }

    return written;
}

static bool use_storage = true;
#define MAX_FILES 10
static int recent_file_size_updated = 0;
static uint8_t heartbeat_count = 0;
void update_file_size()
{
    // P14: 更新所有 10 个文件的大小到 file_num_array[0..9]。
    // 不存在的文件 get_file_size() 返回 0（fs_stat 失败），
    // 这样 BLE 端能一次拿到全部文件大小，识别新录音。
    for (int i = 0; i < MAX_AUDIO_FILES; i++) {
        file_num_array[i] = get_file_size(i + 1);
    }
    // P14b: offset 槽在这里预填（pusher 线程上下文，可安全做文件 IO），
    // storage_read_characteristic 只读内存，不在 BT 线程碰 FAT 锁。
    file_num_array[10] = (uint32_t) get_offset();
    // LOG_PRINTK("file size for file count %d %d\n",file_count,file_num_array[0]);
    // LOG_PRINTK("offset for file count %d %d\n",file_count,file_num_array[1]);
}

void pusher(void)
{
    k_msleep(500);
    while (1) {
        //
        // Load current connection
        //
        struct bt_conn *conn = current_connection;
        // updating the most recent file size is expensive!
        static bool file_size_updated = true;
        static bool connection_was_true = false;
        if (conn && !connection_was_true) {
            k_msleep(100);
            file_size_updated = false;
            connection_was_true = true;
        } else if (!conn) {
            connection_was_true = false;
        }
        if (!file_size_updated) {
            LOG_PRINTK("updating file size\n");
            update_file_size();

            file_size_updated = true;
        }
        if (conn) {
            conn = bt_conn_ref(conn);
        }
        bool valid = true;
        if (current_mtu < MINIMAL_PACKET_SIZE) {
            valid = false;
        } else if (!conn) {
            valid = false;
        } else {
            valid = bt_gatt_is_subscribed(conn, &audio_service.attrs[1], BT_GATT_CCC_NOTIFY); // Check if subscribed
        }

        // P15: any active recording (manual or auto) always writes to the SD
        // card, independent of BLE connection/subscription - "recordings must
        // land on the TF card regardless of BLE state". Live BLE streaming only
        // happens when NO recording is active AND a central is subscribed.
        if (is_recording()) {
            // P14b: 检查"当前写入文件"（file_count 对应）的大小护栏。
            if (file_count >= 1 && file_count <= MAX_AUDIO_FILES &&
                file_num_array[file_count - 1] < MAX_STORAGE_BYTES) {
                k_mutex_lock(&write_sdcard_mutex, K_FOREVER);
                if (is_sd_on()) {
                    // Deferred file switch: start_new_recording was requested
                    // from IRQ/button context and must run in this thread.
                    if (rec_needs_new_file) {
                        rec_needs_new_file = false;
                        start_new_recording();
                        rec_file_bytes = 0;
                    }
                    uint32_t written = write_to_storage();
                    if (written > 0) {
                        rec_file_bytes += written;
                        heartbeat_count++;
                        if (heartbeat_count == 255) {
                            update_file_size();
                            heartbeat_count = 0;
                            LOG_PRINTK("drawing\n");
                        }
                        // Truncation: rotate to a fresh file when the current
                        // one reaches the size cap, keeping the recording going.
                        if (rec_file_bytes >= MAX_AUDIO_FILE_SIZE) {
                            start_new_recording();
                            rec_file_bytes = 0;
                        }
                    }
                }
                k_mutex_unlock(&write_sdcard_mutex);
            }
            if (!is_sd_on() && !ring_buf_is_empty(&ring_buf)) {
                // No SD card: discard stale audio frames so the ring buffer never fills.
                read_from_tx_queue();
            }
        } else if (valid) {
            bool sent = push_to_gatt(conn);
            if (!sent) {
                // k_sleep(K_MSEC(50));
            }
        } else if (!ring_buf_is_empty(&ring_buf)) {
            // Not recording and not subscribed: audio has no consumer. Drain the
            // ring buffer so it never fills up (a full ring buffer triggers an
            // error-log flood that starves the CPU and resets the watchdog).
            read_from_tx_queue();
        }
        if (conn) {
            bt_conn_unref(conn);
        }

        // P14b: k_yield() only hands the CPU to threads of the SAME or higher
        // priority. As the sole PREEMPT(7) thread it span the CPU forever and
        // starved the K_LOWEST (31) SD mount thread: the 500ms head start at
        // pusher boot was its ONLY window - a slow disk_access_init (>500ms,
        // flaky fly-wired card) meant the mount never completed and no mount
        // logs ever appeared. k_msleep(1) yields to ALL lower-priority threads
        // too, while adding at most 1ms latency to the 10ms audio cadence.
        k_msleep(1);
    }
}
extern struct bt_gatt_service storage_service;
//
// Public functions
//
int bt_off()
{
    // First disconnect any active connections
    if (current_connection != NULL) {
        bt_conn_disconnect(current_connection, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(current_connection);
        current_connection = NULL;
    }

    // Stop advertising
    int err = bt_le_adv_stop();
    if (err) {
        LOG_ERR("Failed to stop Bluetooth advertising %d", err);
    }

    // Disable Bluetooth
    err = bt_disable();
    if (err) {
        LOG_ERR("Failed to disable Bluetooth %d", err);
    }

    // Turn off other peripherals
    k_mutex_lock(&write_sdcard_mutex, K_FOREVER);
    sd_off();
    k_mutex_unlock(&write_sdcard_mutex);
    mic_off();

    // Ensure all Bluetooth resources are cleaned up
    is_connected = false;
    storage_is_on = false;
    current_mtu = 0;

    return 0;
}
int bt_on()
{
    int err = bt_enable(NULL);
    bt_le_adv_start(BT_LE_ADV_CONN, bt_ad, ARRAY_SIZE(bt_ad), bt_sd, ARRAY_SIZE(bt_sd));
    bt_gatt_service_register(&storage_service);
    sd_on();
    mic_on();

    return 0;
}

// periodic advertising
int transport_start()
{
    k_mutex_init(&write_sdcard_mutex);

    // Configure callbacks
    bt_conn_cb_register(&_callback_references);
    bt_gatt_cb_register(&_gatt_callbacks);

    // Enable Bluetooth
    int err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Transport bluetooth init failed (err %d)", err);
        return err;
    }
    LOG_INF("Transport bluetooth initialized");

    //  Enable button
#ifdef CONFIG_OMI_ENABLE_BUTTON
    register_button_service();
#endif

#ifdef CONFIG_OMI_ENABLE_SPEAKER
    register_speaker_service();
#endif

    // Start advertising
    memset(storage_temp_data, 0, OPUS_PADDED_LENGTH * 4);
    err = bt_gatt_service_register(&storage_service);
    if (err) {
        LOG_ERR("Failed to register storage service (err %d)", err);
    } else {
        LOG_INF("Storage service registered");
    }
    err = bt_gatt_service_register(&audio_service);
    if (err) {
        LOG_ERR("Failed to register audio service (err %d)", err);
    } else {
        LOG_INF("Audio service registered");
    }
    err = bt_gatt_service_register(&dfu_service);
    if (err) {
        LOG_ERR("Failed to register DFU service (err %d)", err);
    } else {
        LOG_INF("DFU service registered");
    }
    err = bt_le_adv_start(BT_LE_ADV_CONN, bt_ad, ARRAY_SIZE(bt_ad), bt_sd, ARRAY_SIZE(bt_sd));
    if (err) {
        LOG_ERR("Transport advertising failed to start (err %d)", err);
        return err;
    } else {
        LOG_INF("Advertising successfully started");
    }

    int battErr = 0;
    battErr |= battery_init();
    battErr |= battery_charge_start();
    if (battErr) {
        LOG_ERR("Battery init failed (err %d)", battErr);
    } else {
        LOG_INF("Battery initialized");
    }

    // Start pusher
    ring_buf_init(&ring_buf, sizeof(tx_queue), tx_queue);
    k_thread_create(&pusher_thread,
                    pusher_stack,
                    K_THREAD_STACK_SIZEOF(pusher_stack),
                    (k_thread_entry_t) pusher,
                    NULL,
                    NULL,
                    NULL,
                    K_PRIO_PREEMPT(7),
                    0,
                    K_NO_WAIT);

    return 0;
}

struct bt_conn *get_current_connection()
{
    return current_connection;
}

int broadcast_audio_packets(uint8_t *buffer, size_t size)
{
    int retry_count = 0;
    const int max_retries = 3;

    while (retry_count < max_retries && !write_to_tx_queue(buffer, size)) {
        k_sleep(K_MSEC(1));
        retry_count++;
    }

    if (retry_count >= max_retries) {
        LOG_ERR("Failed to write to tx queue after %d retries", max_retries);
        return -1;
    }

    return 0;
}

void accel_off()
{
    gpio_pin_set_dt(&accel_gpio_pin, 0);
}
