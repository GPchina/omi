#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "button.h"
#include "codec.h"
#include "config.h"
#include "led.h"
#include "mic.h"
#include "sdcard.h"
#include "speaker.h"
#include "storage.h"
#include "transport.h"
#include "usb.h"
#include "utils.h"
#include "wdog_facade.h"
#define BOOT_BLINK_DURATION_MS 600
#define BOOT_PAUSE_DURATION_MS 200
#define VBUS_DETECT (1U << 20)
#define WAKEUP_DETECT (1U << 16)
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static void codec_handler(uint8_t *data, size_t len)
{
#ifdef CONFIG_OMI_ENABLE_OFFLINE_STORAGE
    // P13: only feed the transport when someone consumes the audio - a BLE
    // central is connected, or a manual recording (button single tap) is
    // active. When idle standalone, skip so the tx ring buffer does not fill
    // up and drop every packet with error-log spam.
    if (!should_capture_audio()) {
        return;
    }
#endif
    int err = broadcast_audio_packets(data, len);
    if (err) {
        LOG_ERR("Failed to broadcast audio packets: %d", err);
    }
}

static void mic_handler(int16_t *buffer)
{
    int err = codec_receive_pcm(buffer, MIC_BUFFER_SAMPLES);
    if (err) {
        LOG_ERR("Failed to process PCM data: %d", err);
    }
}

#ifdef CONFIG_OMI_ENABLE_OFFLINE_STORAGE
// P13b: SD mount worker thread. See the call site in main() for the rationale.
K_THREAD_STACK_DEFINE(sd_mount_stack, 8192);
static struct k_thread sd_mount_thread_data;

static void sd_mount_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    // P13c: retry the mount periodically instead of giving up. A card that
    // wasn't seated (or a cold SD module on USB power) will start answering
    // once power/re-seating is fixed, and the card should mount without a
    // reboot. Logs a line on each transition so the serial tells us clearly.
    bool reported_failure = false;
    while (1) {
        int err = mount_sd_card();
        if (!err) {
            if (reported_failure) {
                LOG_INF("SD mount OK (recovered)\n");
            }
            k_msleep(500);
            LOG_INF("Initializing storage...\n");
            err = storage_init();
            if (err) {
                LOG_ERR("storage_init failed (err %d)", err);
            }
            return;  // storage ready; worker thread exits
        }
        if (!reported_failure) {
            LOG_ERR("SD mount failed (err %d) - retrying every 5s (BLE stays up)", err);
            reported_failure = true;
        }
        k_msleep(5000);
    }
}
#endif

void bt_ctlr_assert_handle(char *name, int type)
{
    // Crash beacon: the SoftDevice Controller asserted (suspected llcp stall
    // during connect-time procedures). The deferred log thread is typically
    // already wedged by then, so a log line would be invisible. Blink the red
    // LED fast forever instead, feeding the watchdog so the pattern persists
    // and is clearly distinct from a watchdog reset (which would stop it).
    LOG_INF("Bluetooth assert: %s (type %d)", name ? name : "NULL", type);
    while (1) {
        set_led_red(true);
        k_busy_wait(100 * 1000);
        set_led_red(false);
        k_busy_wait(100 * 1000);
        watchdog_feed();
    }
}

static void print_reset_reason(void)
{
    uint32_t reas = NRF_POWER->RESETREAS;

    // Clear the reset reason register
    NRF_POWER->RESETREAS = reas;

    if (reas & POWER_RESETREAS_DOG_Msk) {
        printk("Reset by WATCHDOG\n");
    } else if (reas & POWER_RESETREAS_NFC_Msk) {
        printk("Wake up by NFC field detect\n");
    } else if (reas & POWER_RESETREAS_RESETPIN_Msk) {
        printk("Reset by pin-reset\n");
    } else if (reas & POWER_RESETREAS_SREQ_Msk) {
        printk("Reset by soft-reset\n");
    } else if (reas & POWER_RESETREAS_LOCKUP_Msk) {
        printk("Reset by CPU LOCKUP\n");
    } else if (reas) {
        printk("Reset by a different source (0x%08X)\n", reas);
    } else {
        printk("Power-on-reset\n");
    }
}

bool is_connected = false;
bool is_charging = false;
extern bool is_off;
extern bool usb_charge;
static void boot_led_sequence(void)
{
    // Red blink
    set_led_red(true);
    k_msleep(BOOT_BLINK_DURATION_MS);
    set_led_red(false);
    k_msleep(BOOT_PAUSE_DURATION_MS);
    // Green blink
    set_led_green(true);
    k_msleep(BOOT_BLINK_DURATION_MS);
    set_led_green(false);
    k_msleep(BOOT_PAUSE_DURATION_MS);
    // Blue blink
    set_led_blue(true);
    k_msleep(BOOT_BLINK_DURATION_MS);
    set_led_blue(false);
    k_msleep(BOOT_PAUSE_DURATION_MS);
    // All LEDs on
    set_led_red(true);
    set_led_green(true);
    set_led_blue(true);
    k_msleep(BOOT_BLINK_DURATION_MS);
    // All LEDs off
    set_led_red(false);
    set_led_green(false);
    set_led_blue(false);
}

void set_led_state()
{
    // Recording and connected state - BLUE

    if (usb_charge) {
        is_charging = !is_charging;
        if (is_charging) {
            set_led_green(true);
        } else {
            set_led_green(false);
        }
    } else {
        set_led_green(false);
    }
    if (is_off) {
        set_led_red(false);
        set_led_blue(false);
        return;
    }
#ifdef CONFIG_OMI_ENABLE_OFFLINE_STORAGE
    // P13: manual recording indicator (standalone) - blink blue while
    // recording to SD, distinguishable from idle (solid red) and from
    // BLE-connected (solid blue).
    if (!is_connected && is_manual_recording()) {
        static bool rec_blink = false;
        rec_blink = !rec_blink;
        set_led_blue(rec_blink);
        set_led_red(false);
        return;
    }
#endif
    if (is_connected) {
        set_led_blue(true);
        set_led_red(false);
        return;
    }

    // Recording but lost connection - RED
    if (!is_connected) {
        set_led_red(true);
        set_led_blue(false);
        return;
    }
}

int main(void)
{
    int err;

    // Print and clear reset reason
    print_reset_reason();

    NRF_POWER->DCDCEN = 1;
    NRF_POWER->DCDCEN0 = 1;

    LOG_INF("Booting...\n");

    // Firmware version fingerprint: printed at boot AND shown as blue LED
    // blink count, so the running build can be verified beyond doubt even
    // when the log pipe dies later. Bump on every patch!
    LOG_INF("FW VERSION: P14F\n");

    LOG_INF("Model: %s", CONFIG_BT_DIS_MODEL);
    LOG_INF("Firmware revision: %s", CONFIG_BT_DIS_FW_REV_STR);
    LOG_INF("Hardware revision: %s", CONFIG_BT_DIS_HW_REV_STR);
    // Force QSPI flash into deep sleep mode
    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(p25q16h));
    if (device_is_ready(flash_dev)) {
        err = pm_device_action_run(flash_dev, PM_DEVICE_ACTION_SUSPEND);
        if (err) {
            LOG_ERR("Failed to suspend QSPI flash: %d", err);
        }
    } else {
        LOG_ERR("QSPI flash device not ready");
    }
    LOG_PRINTK("\n");
    LOG_INF("Initializing LEDs...\n");

    err = led_start();
    if (err) {
        LOG_ERR("Failed to initialize LEDs (err %d)", err);
        return err;
    }

    // Version blink: blue LED blinks 24 times = patch 14f. Visible without any
    // serial connection, distinguishes builds when flashing mistakes happen.
    for (int i = 0; i < 24; i++) {
        set_led_blue(true);
        k_msleep(120);
        set_led_blue(false);
        k_msleep(120);
    }

    // Run the boot LED sequence
    boot_led_sequence();

    // Initialize watchdog early to catch any freezes during boot
    err = watchdog_init();
    if (err) {
        LOG_WRN("Watchdog init failed (err %d), continuing without watchdog", err);
    }

    // Enable battery
#ifdef CONFIG_OMI_ENABLE_BATTERY
    err = battery_init();
    if (err) {
        LOG_ERR("Battery init failed (err %d)", err);
        return err;
    }

    err = battery_charge_start();
    if (err) {
        LOG_ERR("Battery failed to start (err %d)", err);
        return err;
    }
    LOG_INF("Battery initialized");
#endif

    // Enable button
#ifdef CONFIG_OMI_ENABLE_BUTTON
    err = button_init();
    if (err) {
        LOG_ERR("Failed to initialize Button (err %d)", err);
        return err;
    }
    LOG_INF("Button initialized");
    activate_button_work();
#endif

    // Enable accelerometer
#ifdef CONFIG_OMI_ENABLE_ACCELEROMETER
    err = accel_start();
    if (err) {
        LOG_ERR("Accelerometer failed to activated (err %d)", err);
        return err;
    }
    LOG_INF("Accelerometer initialized");
#endif

    // Enable speaker
#ifdef CONFIG_OMI_ENABLE_SPEAKER
    err = speaker_init();
    if (err) {
        LOG_ERR("Speaker failed to start");
        return err;
    }
    LOG_INF("Speaker initialized");
#endif

    // Enable sdcard
#ifdef CONFIG_OMI_ENABLE_OFFLINE_STORAGE
    LOG_PRINTK("\n");
    // P13b: mount in a dedicated lowest-priority thread. On the fly-wired
    // module disk_access_init() was observed to spin without ever yielding,
    // starving the (lower) log thread and the watchdog feed in the main loop
    // below -> 30s watchdog reset boot loop, BLE never came up. At lowest
    // priority with timeslicing the worst case is wasted CPU: boot, BLE and
    // buttons keep working and the mount result is still visible on serial.
    LOG_INF("Mounting SD card in background thread...\n");
    k_thread_create(&sd_mount_thread_data, sd_mount_stack,
                    K_THREAD_STACK_SIZEOF(sd_mount_stack),
                    sd_mount_thread_fn, NULL, NULL, NULL,
                    K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
#endif

    // Enable haptic
#ifdef CONFIG_OMI_ENABLE_HAPTIC
    LOG_PRINTK("\n");
    LOG_INF("Initializing haptic...\n");

    err = init_haptic_pin();
    if (err) {
        LOG_ERR("Failed to initialize haptic pin (err %d)", err);
        return err;
    }
    LOG_INF("Haptic pin initialized");
#endif

    // Enable usb
#ifdef CONFIG_OMI_ENABLE_USB
    LOG_PRINTK("\n");
    LOG_INF("Initializing power supply check...\n");

    err = init_usb();
    if (err) {
        LOG_ERR("Failed to initialize power supply (err %d)", err);
        return err;
    }
#endif

    // Indicate transport initialization
    LOG_PRINTK("\n");
    LOG_INF("Initializing transport...\n");

    set_led_green(true);
    set_led_green(false);

    // Start transport
    int transportErr;
    transportErr = transport_start();
    if (transportErr) {
        LOG_ERR("Failed to start transport (err %d)", transportErr);
        // TODO: Detect the current core is app core or net core
        // Blink green LED to indicate error
        for (int i = 0; i < 5; i++) {
            set_led_green(!gpio_pin_get_dt(&led_green));
            k_msleep(200);
        }
        set_led_green(false);

        return transportErr;
    }

#ifdef CONFIG_OMI_ENABLE_SPEAKER
    play_boot_sound();
#endif

    LOG_PRINTK("\n");
    LOG_INF("Initializing codec...\n");

    set_led_blue(true);

    // Audio codec(opus) callback
    set_codec_callback(codec_handler);
    err = codec_start();
    if (err) {
        LOG_ERR("Failed to start codec: %d", err);
        // Blink blue LED to indicate error
        for (int i = 0; i < 5; i++) {
            set_led_blue(!gpio_pin_get_dt(&led_blue));
            k_msleep(200);
        }
        set_led_blue(false);
        return err;
    }

#ifdef CONFIG_OMI_ENABLE_HAPTIC
    play_haptic_milli(500);
#endif
    set_led_blue(false);

    // Indicate microphone initialization
    LOG_PRINTK("\n");
    LOG_INF("Initializing microphone...\n");

    set_led_red(true);
    set_led_green(true);

    set_mic_callback(mic_handler);
    err = mic_start();
    if (err) {
        LOG_ERR("Failed to start microphone: %d", err);
        // Blink red and green LEDs to indicate error
        for (int i = 0; i < 5; i++) {
            set_led_red(!gpio_pin_get_dt(&led_red));
            set_led_green(!gpio_pin_get_dt(&led_green));
            k_msleep(200);
        }
        set_led_red(false);
        set_led_green(false);
        return err;
    }

    set_led_red(false);
    set_led_green(false);

    // Indicate successful initialization
    LOG_PRINTK("\n");
    LOG_INF("Device initialized successfully\n");

    // Feed the watchdog during the boot demo sequence: the only regular feed is
    // in the main loop below. With no SD card the pre-pusher error burst can
    // wedge the console and hang this section past the 30s timeout.
    watchdog_feed();
    set_led_blue(true);
    k_msleep(1000);
    watchdog_feed();
    set_led_blue(false);

    // Main loop
    LOG_PRINTK("\n");
    LOG_INF("Entering main loop...\n");

    while (1) {
        watchdog_feed();

        set_led_state();
        k_msleep(500);
    }

    // Unreachable
    return 0;
}
