#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
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

// P15: unified recording state machine (manual button / auto VAD).
// Replaces the P13 manual_record_on bool. All recording - manual or auto -
// writes to the SD card regardless of BLE state. Manual is button-toggled and
// VAD-independent; auto is voice-activated and ends after 10s of silence.
typedef enum {
    REC_IDLE = 0,
    REC_MANUAL,   // manual recording: button toggled, VAD-independent
    REC_AUTO      // auto recording: voice-activated, ends after 10s silence
} rec_mode_t;

// True while any recording (manual or auto) is active.
bool is_recording(void);
rec_mode_t get_rec_mode(void);
// Single tap handler: start/stop manual recording (also switches auto->manual).
void record_button_toggle(void);
// Feed raw PCM for VAD (called every 100ms by mic_handler).
void record_feed_pcm(const int16_t *samples, size_t count);
// True when captured audio has a consumer (BLE central or active recording).
bool should_capture_audio(void);

void accel_off();
#endif