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
#include <math.h>                    // P15h: sinf/cosf/sqrtf for spectral VAD
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
// Deferred finalize. Set on rec_stop() (IRQ context) so the pusher thread can
// flush the buffered tail and write the true file size back to file_num_array.
// Without this, start_new_recording() keeps finding the same "size==0" slot
// offline and every auto-recording segment overwrites the previous one.
static volatile bool rec_finalize_pending = false;

// P15f: audio-data notification subscription flag. Set/cleared in the CCC
// handler and cleared again on disconnect, so the codec only produces frames
// when a central has actually subscribed to the audio notifications. A central
// that merely connects (e.g. the autosync file-transfer client) never enables
// audio notify - without this flag, is_connected alone made the codec fill the
// tx ring buffer and starve the lower-prio pusher, flooding "tx queue after 3
// retries". Volatile: read from the codec thread, written from the BLE RX
// context (single-core nRF52, aligned bool access is atomic).
static volatile bool audio_notify_subscribed = false;

// P15h: spectral voice activity detection. Four frequency-domain features
// computed per 100 ms PCM frame replace the time-domain energy threshold
// (P15e/P15g): a single RMS level cannot tell speech apart from a TV/music
// or a fan running in the background - they have the same total energy.
//
//   f1 voice-band share   E(300-3400 Hz) / E(0-8000 Hz)
//                         speech -> ~0.25-0.40 (low-pass mic),  HVAC/white
//                         noise have most energy outside the band.
//   f2 high-freq share    E(>4000 Hz) / E(300-3400 Hz)
//                         speech -> very small (~0.02), anything with hiss
//                         or broadband noise climbs above 0.2.
//   f3 harmonic peaks     count of bins > 4x median in the F0..3F0 band
//                         (60-1000 Hz, after the speaker's fundamental).
//                         speech has F0 + 2F0 + 3F0 -> 2-3 peaks; pure-tone
//                         noise (fan, tone) -> 0-1 peak.
//   f4 temporal pulse     max energy in the last 8 frames / min energy in
//                         the last 8 frames. Speech "pulses" between
//                         syllable-on and syllable-off (>50 dB swings), so
//                         f4 > 8. Continuous TV / fan / HVAC stay smooth,
//                         f4 < 5. THIS IS THE KILLER FEATURE: it kills the
//                         user's exact complaint (continuous noise waking).
//
// Rule: 3 of 4 features positive + 200 ms (2-frame) start debounce.
//
// PoC (`p13_tmp/vad_poc_v4.py`) measured on a real Omi recording + 5
// synthesised background noises:
//                                v4 (this scheme)   v3 (loose)   energy
//   real voice                  ~34%               58%          -
//   white noise                  0%                0%           yes
//   fan / motor hum              0%                0%           yes
//   HVAC hiss                   0%                0%           yes
//   cough / sneeze              0%                0%           yes
//   TV / music                  1%                39%          yes
//
// The 34% voice "recall" sounds low but is fine: speech is full of silence
// gaps (between words and syllables) that the rule must not flag as voice.
// Once the second voice frame fires the debounce latch, the recorder runs
// for the whole utterance (10 s silence timeout), so what matters is that
// the rule fires within the first word.
#define FFT_N              256    // 16 kHz sampling -> 62.5 Hz / bin
#define FFT_BIN_HZ         62.5f  // = sr / FFT_N
#define VAD_HISTORY_FRAMES 8
// Bin index = Hz / FFT_BIN_HZ. Use round-to-nearest.
#define VAD_BIN_LO      5    // ~300 Hz   (start of voice band)
#define VAD_BIN_VO_HI  54    // ~3400 Hz  (end of voice band)
#define VAD_BIN_HI     64    // ~4000 Hz  (start of high-band noise band)
#define VAD_F0_LO       1    // ~60  Hz   (start of F0 search)
#define VAD_F0_HI      16    // ~1000 Hz  (end of F0 / start of harmonics)
// Spectral VAD feature thresholds (PoC-tuned). Const, no per-room tuning.
#define VAD_F1_THR     0.20f  // voice-band share minimum
#define VAD_F2_THR     0.15f  // high-freq share maximum
#define VAD_F3_THR     2      // harmonic peak count minimum
#define VAD_F4_THR     8.0f   // temporal pulse ratio minimum
#define VAD_VOTES_REQ  3      // >=3 of 4 features must be positive
// Auto recording ends after 10 s of continuous silence.
#define VAD_SILENCE_TIMEOUT_MS 10000
// 2 frames (200 ms) of consecutive voice-vote before latching REC_AUTO.
// PoC showed this kills cough / sneeze / door-slam while still letting real
// speech in.
#define VAD_START_DEBOUNCE_FRAMES 2
// Single-file truncation: ~30 s @ 32 kbps Opus (~4 KB/s). Keeps each
// recording short enough for fast medium-model transcription and BLE
// download.
#define MAX_AUDIO_FILE_SIZE 120000

// 256-point symmetric Hann window, precomputed. 1 KB ROM.
static const float hann_window[FFT_N] = {
    0.000000f, 0.000152f, 0.000607f, 0.001365f,
    0.002427f, 0.003790f, 0.005454f, 0.007419f,
    0.009683f, 0.012244f, 0.015102f, 0.018253f,
    0.021698f, 0.025433f, 0.029455f, 0.033764f,
    0.038355f, 0.043227f, 0.048376f, 0.053800f,
    0.059494f, 0.065456f, 0.071681f, 0.078166f,
    0.084908f, 0.091902f, 0.099143f, 0.106628f,
    0.114351f, 0.122309f, 0.130496f, 0.138907f,
    0.147537f, 0.156382f, 0.165435f, 0.174691f,
    0.184144f, 0.193790f, 0.203621f, 0.213632f,
    0.223818f, 0.234170f, 0.244684f, 0.255354f,
    0.266171f, 0.277131f, 0.288226f, 0.299449f,
    0.310794f, 0.322255f, 0.333823f, 0.345491f,
    0.357254f, 0.369104f, 0.381032f, 0.393033f,
    0.405099f, 0.417223f, 0.429397f, 0.441614f,
    0.453866f, 0.466146f, 0.478447f, 0.490761f,
    0.503080f, 0.515398f, 0.527706f, 0.539997f,
    0.552264f, 0.564500f, 0.576696f, 0.588845f,
    0.600941f, 0.612976f, 0.624941f, 0.636832f,
    0.648638f, 0.660355f, 0.671974f, 0.683489f,
    0.694893f, 0.706178f, 0.717338f, 0.728366f,
    0.739256f, 0.750000f, 0.760592f, 0.771027f,
    0.781296f, 0.791395f, 0.801317f, 0.811056f,
    0.820607f, 0.829962f, 0.839118f, 0.848067f,
    0.856805f, 0.865327f, 0.873626f, 0.881699f,
    0.889540f, 0.897145f, 0.904508f, 0.911626f,
    0.918495f, 0.925109f, 0.931464f, 0.937558f,
    0.943387f, 0.948946f, 0.954233f, 0.959243f,
    0.963976f, 0.968426f, 0.972592f, 0.976471f,
    0.980061f, 0.983359f, 0.986364f, 0.989074f,
    0.991487f, 0.993601f, 0.995416f, 0.996930f,
    0.998142f, 0.999052f, 0.999659f, 0.999962f,
    0.999962f, 0.999659f, 0.999052f, 0.998142f,
    0.996930f, 0.995416f, 0.993601f, 0.991487f,
    0.989074f, 0.986364f, 0.983359f, 0.980061f,
    0.976471f, 0.972592f, 0.968426f, 0.963976f,
    0.959243f, 0.954233f, 0.948946f, 0.943387f,
    0.937558f, 0.931464f, 0.925109f, 0.918495f,
    0.911626f, 0.904508f, 0.897145f, 0.889540f,
    0.881699f, 0.873626f, 0.865327f, 0.856805f,
    0.848067f, 0.839118f, 0.829962f, 0.820607f,
    0.811056f, 0.801317f, 0.791395f, 0.781296f,
    0.771027f, 0.760592f, 0.750000f, 0.739256f,
    0.728366f, 0.717338f, 0.706178f, 0.694893f,
    0.683489f, 0.671974f, 0.660355f, 0.648638f,
    0.636832f, 0.624941f, 0.612976f, 0.600941f,
    0.588845f, 0.576696f, 0.564500f, 0.552264f,
    0.539997f, 0.527706f, 0.515398f, 0.503080f,
    0.490761f, 0.478447f, 0.466146f, 0.453866f,
    0.441614f, 0.429397f, 0.417223f, 0.405099f,
    0.393033f, 0.381032f, 0.369104f, 0.357254f,
    0.345491f, 0.333823f, 0.322255f, 0.310794f,
    0.299449f, 0.288226f, 0.277131f, 0.266171f,
    0.255354f, 0.244684f, 0.234170f, 0.223818f,
    0.213632f, 0.203621f, 0.193790f, 0.184144f,
    0.174691f, 0.165435f, 0.156382f, 0.147537f,
    0.138907f, 0.130496f, 0.122309f, 0.114351f,
    0.106628f, 0.099143f, 0.091902f, 0.084908f,
    0.078166f, 0.071681f, 0.065456f, 0.059494f,
    0.053800f, 0.048376f, 0.043227f, 0.038355f,
    0.033764f, 0.029455f, 0.025433f, 0.021698f,
    0.018253f, 0.015102f, 0.012244f, 0.009683f,
    0.007419f, 0.005454f, 0.003790f, 0.002427f,
    0.001365f, 0.000607f, 0.000152f, 0.000000f,
};

// P15h: spectral VAD working state. volatile so reads in the IRQ context
// (record_feed_pcm) and writes in the same context see a consistent view.
// All single-core nRF52, 32-bit aligned accesses are atomic; bool is 1 byte.
static volatile float energy_history[VAD_HISTORY_FRAMES] = {0};
static volatile uint32_t energy_history_fill = 0; // number of frames stored
static volatile bool    energy_history_full  = false;
// Count of consecutive "voice" votes in the IRQ - debounce latch.
static volatile uint8_t rec_vote_streak = 0;

// P15h: FFT scratch buffers. Made file-scope (not stack-allocated) so we do
// not blow the ~2 KB nRF52 IRQ stack. The PDM handler calls mic_handler in
// interrupt context, which calls record_feed_pcm, which calls
// compute_spectral_features. compute_spectral_features previously allocated
// ~2.8 KB of stack (buf + mag2 + band + sorted) - enough to overflow the
// default CONFIG_ISR_STACK_SIZE=2048. Static allocation makes this safe
// regardless of IRQ stack size. Single-core nRF52 means no race.
static float fft_buf[FFT_N * 2];        // 2 KB
static float fft_mag2[FFT_N / 2 + 1];   // 516 B
static float fft_band[16];              // harmonic peak search band (64 B)
static float fft_sorted[16];            // median sort scratch (64 B)

// In-place radix-2 complex FFT on a length-N buffer laid out as
// [re0, im0, re1, im1, ...]. After the call, the buffer holds the spectrum.
// We use Zephyr's newlib <math.h> for sinf/cosf. Each FFT call here is 256
// points with 8 stages, ~2k complex multiplies; well under the 100 ms frame
// budget on the nRF52 with its single-precision FPU.
static void fft_inplace_f32(float *buf, int n)
{
    // Bit-reversal reordering
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; (j & bit); bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float tr = buf[i * 2];
            buf[i * 2] = buf[j * 2];
            buf[j * 2] = tr;
            float ti = buf[i * 2 + 1];
            buf[i * 2 + 1] = buf[j * 2 + 1];
            buf[j * 2 + 1] = ti;
        }
    }
    // Butterflies
    for (int s = 1; (1 << s) <= n; s++) {
        int m = 1 << s;
        int m2 = m >> 1;
        for (int k = 0; k < n; k += m) {
            for (int j = 0; j < m2; j++) {
                float angle = -6.28318530718f * (float)j / (float)m;
                float wr = cosf(angle);
                float wi = sinf(angle);
                int a = (k + j) * 2;
                int b = (k + j + m2) * 2;
                float tr = wr * buf[b] - wi * buf[b + 1];
                float ti = wr * buf[b + 1] + wi * buf[b];
                buf[b]     = buf[a]     - tr;
                buf[b + 1] = buf[a + 1] - ti;
                buf[a]     = buf[a]     + tr;
                buf[a + 1] = buf[a + 1] + ti;
            }
        }
    }
}

// Compute the four spectral VAD features for one 100 ms block. Samples are
// raw int16 from the PDM (we do not subtract DC bias - the spectral features
// are ratios and DC bias cancels in f4's max/min temporal ratio). We work in
// float (nRF52 has FPU) and accumulate in float.
static void compute_spectral_features(const int16_t *samples, size_t count,
                                      bool *out_vote, bool *out_voice_vote)
{
    // Pull a 256-sample window from the middle of the 1600-sample block.
    // Using the centre (not the leading edge) means the first ~700 ms of
    // audio is not lost while we fill FFT windows.
    size_t centre = count / 2;
    size_t half = FFT_N / 2;
    size_t start = (centre > half) ? (centre - half) : 0;
    if (start + FFT_N > count) {
        start = (count > FFT_N) ? (count - FFT_N) : 0;
    }
    // Windowed complex input buffer (real part = windowed sample, imag = 0).
    // fft_buf and fft_mag2 are file-scope statics (declared above) so this
    // function does not allocate ~2 KB on the IRQ stack.
    for (int i = 0; i < FFT_N; i++) {
        size_t idx = start + i;
        float s = (idx < count) ? (float)samples[idx] : 0.0f;
        fft_buf[i * 2]     = s * hann_window[i];
        fft_buf[i * 2 + 1] = 0.0f;
    }
    fft_inplace_f32(fft_buf, FFT_N);

    // Magnitude squared per bin (FFT_N/2+1 bins; DC at 0 is ignored).
    fft_mag2[0] = 0.0f;
    for (int i = 1; i <= FFT_N / 2; i++) {
        float re = fft_buf[i * 2];
        float im = fft_buf[i * 2 + 1];
        fft_mag2[i] = re * re + im * im;
    }

    // Band energies
    float e_lo  = 0.0f, e_voice = 0.0f, e_hi = 0.0f;
    for (int b = 1; b < VAD_BIN_LO; b++)               e_lo   += fft_mag2[b];
    for (int b = VAD_BIN_LO; b < VAD_BIN_VO_HI; b++)   e_voice += fft_mag2[b];
    for (int b = VAD_BIN_VO_HI; b < VAD_BIN_HI; b++)   e_hi   += fft_mag2[b];
    // Top-bin remainder also counts as "high" so a steep spectral roll-off
    // does not artificially lower f2.
    for (int b = VAD_BIN_HI; b <= FFT_N / 2; b++)    e_hi   += fft_mag2[b];

    float e_total = e_lo + e_voice + e_hi;
    float f1 = (e_total > 0.0f) ? (e_voice / e_total) : 0.0f;
    float f2 = (e_voice > 0.0f) ? (e_hi / e_voice) : 999.0f;

    // Harmonic peak count in the F0..3F0 band.
    int peak_count = 0;
    {
        // Use sqrt-magnitude for peak detection (auditory-style loudness)
        const int band_n = VAD_F0_HI - VAD_F0_LO + 1;
        for (int i = 0; i < band_n; i++) {
            fft_band[i] = sqrtf(fft_mag2[VAD_F0_LO + i] + 1e-12f);
        }
        // Median for "noise floor" reference
        for (int i = 0; i < band_n; i++) fft_sorted[i] = fft_band[i];
        // tiny insertion sort (n is <= 16)
        for (int i = 1; i < band_n; i++) {
            float v = fft_sorted[i];
            int j = i - 1;
            while (j >= 0 && fft_sorted[j] > v) { fft_sorted[j + 1] = fft_sorted[j]; j--; }
            fft_sorted[j + 1] = v;
        }
        float med = fft_sorted[band_n / 2];
        float thr = med * 4.0f;
        if (thr > 0.0f) {
            for (int i = 0; i < band_n; i++) {
                if (fft_band[i] > thr) {
                    peak_count++;
                    // Skip adjacent bins (spectral leakage from one peak
                    // shows up in 1-2 neighbors).
                    i += 2;
                }
            }
        }
    }

    // Total energy for the temporal pulse ratio (sum of mag2 across bins).
    float e = e_total;

    // Push into rolling history (FIFO overwriting oldest)
    uint32_t h_idx = energy_history_fill;
    if (energy_history_full) {
        // overwrite in a small ring. Easier: shift up by one.
        for (uint32_t i = 0; i + 1 < VAD_HISTORY_FRAMES; i++) {
            energy_history[i] = energy_history[i + 1];
        }
        energy_history[VAD_HISTORY_FRAMES - 1] = e;
    } else {
        energy_history[h_idx] = e;
        energy_history_fill++;
        if (energy_history_fill >= VAD_HISTORY_FRAMES) {
            energy_history_full = true;
            energy_history_fill = VAD_HISTORY_FRAMES;
        }
    }

    // Temporal pulse ratio: max / min across the history window.
    float f4 = 1.0f;
    uint32_t n = energy_history_full ? VAD_HISTORY_FRAMES : energy_history_fill;
    if (n >= 4) {
        float mn = 1e30f, mx = -1e30f;
        for (uint32_t i = 0; i < n; i++) {
            float v = energy_history[i];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
        }
        f4 = mx / (mn + 1e-9f);
    }

    // Combine
    int votes = 0;
    if (f1 > VAD_F1_THR) votes++;
    if (f2 < VAD_F2_THR) votes++;
    if (peak_count >= VAD_F3_THR) votes++;
    if (f4 > VAD_F4_THR) votes++;
    *out_vote = (votes >= VAD_VOTES_REQ);
    *out_voice_vote = *out_vote;
}

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
    rec_finalize_pending = true;   // pusher will flush + refresh file size
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
    if (count == 0) {
        return;
    }

    // P15h: spectral VAD. Compute the four frequency-domain features on this
    // 100 ms block and ask the algorithm whether it is "voice".
    //
    // We do NOT subtract DC bias here: the spectral features are ratios
    // (f1 = voice-band share, f2 = high-freq share, f4 = max/min temporal
    // energy), so an additive PDM DC bias only adds constant energy to the
    // first few bins (we explicitly zero mag2[0]) and cancels in f4's max/min
    // ratio. The old energy-VAD needed DC removal because sumsq was an
    // absolute threshold; ratios are bias-invariant.
    bool voice = false;
    bool voice_vote_unused = false;
    compute_spectral_features(samples, count, &voice, &voice_vote_unused);

    if (rec_mode == REC_MANUAL) {
        return;                        // manual mode: VAD has no say
    }

    if (rec_mode == REC_IDLE) {
        // Debounce: only latch auto recording after a few consecutive voice
        // votes, so a one-frame TV peak / cough can't start REC_AUTO.
        if (voice) {
            if (++rec_vote_streak >= VAD_START_DEBOUNCE_FRAMES) {
                rec_mark_start(REC_AUTO);
                rec_vote_streak = 0;   // ready for next segment
            }
        } else {
            rec_vote_streak = 0;
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
    //   - a central that SUBSCRIBED to the audio notifications (live stream), or
    //   - an active recording (manual or auto) that has a mounted SD card.
    // P15f: the previous `is_connected` gate was too loose - a central that
    // connects but never enables audio notify (the autosync file-transfer
    // client, or the gap between connect and subscribe) has no consumer for the
    // codec output, so producing frames only filled the tx ring buffer and
    // starved the lower-prio pusher thread, flooding "tx queue after 3 retries".
    // Gate on the actual subscription instead.
    return audio_notify_subscribed || (is_recording() && is_sd_on());
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
    // P15f: track the audio-data subscription (attrs[2]) so the codec knows
    // whether a live-stream consumer exists. The speaker CCC (attrs[5]) shares
    // this handler but must NOT toggle audio capture.
    if (value == BT_GATT_CCC_NOTIFY) {
        LOG_INF("Client subscribed for notifications");
        if (attr == &audio_service.attrs[2]) {
            audio_notify_subscribed = true;
        }
    } else if (value == 0) {
        LOG_INF("Client unsubscribed from notifications");
        if (attr == &audio_service.attrs[2]) {
            audio_notify_subscribed = false;
        }
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
    // P15f: CCC state is per-connection and cleared by the stack on disconnect
    // without invoking the CCC write handler, so reset the subscription flag
    // here - otherwise it would linger true and the next connect-without-
    // subscribe would re-flood the tx queue.
    audio_notify_subscribed = false;

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
        // P15c: only count bytes when the write actually lands on the card.
        // write_to_file() now returns 0 on success / negative errno on failure;
        // treating every call as MAX_WRITE_SIZE made rec_file_bytes climb even
        // when fs_open failed, spuriously triggering truncation + "drawing".
        written = (write_to_file(write_ptr, MAX_WRITE_SIZE) == 0) ? MAX_WRITE_SIZE : 0;

        buffer_offset = packet_size;
        storage_temp_data[0] = tx_buffer_size;
        memcpy(storage_temp_data + 1, buffer, tx_buffer_size);

    } else if (buffer_offset + packet_size == MAX_WRITE_SIZE - 1) { // exact frame needed
        storage_temp_data[buffer_offset] = tx_buffer_size;
        memcpy(storage_temp_data + buffer_offset + 1, buffer, tx_buffer_size);
        buffer_offset = 0;
        uint8_t *write_ptr = (uint8_t *) storage_temp_data;
        written = (write_to_file(write_ptr, MAX_WRITE_SIZE) == 0) ? MAX_WRITE_SIZE : 0;

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

// P15 fix: flush the current file's buffered Opus tail and record its true size
// into file_num_array. Called from the pusher thread (safe to do FS IO) at three
// points: (1) rec_stop() finalize, (2) truncation rotation, (3) new-recording
// file allocation. Without this, start_new_recording() keeps finding the same
// "size==0" slot offline (the array is only refreshed by update_file_size() on
// BLE connect / by scan_audio_files() at mount), so every VAD segment and every
// truncation re-selects a01 and recordings clobber each other, and the partial
// 440-byte tail block leaks into the start of the next recording.
static void finalize_current_file(void)
{
    if (buffer_offset > 0) {
        write_to_file(storage_temp_data, buffer_offset);
        buffer_offset = 0;
    }
    if (file_count >= 1 && file_count <= MAX_AUDIO_FILES) {
        file_num_array[file_count - 1] = get_file_size(file_count);
    }
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
                        rec_finalize_pending = false;   // consume any pending stop-finalize
                        finalize_current_file();        // flush old tail + record old size
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
                            finalize_current_file();    // flush tail + record size of OLD file
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
        } else if (rec_finalize_pending) {
            // P15: recording just stopped (rec_stop set this flag from IRQ/button
            // context). Flush the buffered tail and refresh the file size so the
            // next recording allocates a fresh slot instead of re-picking the one
            // just written (the array is otherwise stale while fully offline).
            k_mutex_lock(&write_sdcard_mutex, K_FOREVER);
            if (is_sd_on()) {
                finalize_current_file();
            } else {
                buffer_offset = 0;   // SD gone: discard the unflushable tail
            }
            k_mutex_unlock(&write_sdcard_mutex);
            rec_finalize_pending = false;
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
