#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdbool.h>
#include <zephyr/drivers/sensor.h>
typedef struct sensors {

    struct sensor_value a_x;
    struct sensor_value a_y;
    struct sensor_value a_z;
    struct sensor_value g_x;
    struct sensor_value g_y;
    struct sensor_value g_z;
};
/**
 * @brief Initialize the BLE transport logic
 *
 * Initializes the BLE Logic
 *
 * @return 0 if successful, negative errno code if error
 */
int transport_start();
int broadcast_audio_packets(uint8_t *buffer, size_t size);
struct bt_conn *get_current_connection();
int bt_on();
int bt_off();

// P13: manual (button-triggered) offline recording to the SD card.
bool is_manual_recording(void);
void set_manual_recording(bool on);
// True when captured audio has a consumer (BLE central or manual recording).
bool should_capture_audio(void);

void accel_off();
#endif