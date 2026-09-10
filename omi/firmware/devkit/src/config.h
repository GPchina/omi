#include <haly/nrfy_gpio.h>

// #define SAMPLE_RATE 16000
#define MIC_GAIN 64
#define MIC_IRC_PRIORITY 7
#define MIC_BUFFER_SAMPLES 1600    // 100ms
#define AUDIO_BUFFER_SAMPLES 16000 // 1s
#define NETWORK_RING_BUF_SIZE 32   // number of frames * CODEC_OUTPUT_MAX_BYTES
#define MINIMAL_PACKET_SIZE 100    // Less than that doesn't make sence to send anything at all

// PIN definitions
// https://github.com/Seeed-Studio/Adafruit_nRF52_Arduino/blob/5aa3573913449410fd60f76b75673c53855ff2ec/variants/Seeed_XIAO_nRF52840_Sense/variant.cpp#L34
#define PDM_DIN_PIN NRF_GPIO_PIN_MAP(0, 16)
#define PDM_CLK_PIN NRF_GPIO_PIN_MAP(1, 0)
#define PDM_PWR_PIN NRF_GPIO_PIN_MAP(1, 10)

// Codecs
#ifdef CONFIG_OMI_CODEC_OPUS
#define CODEC_OPUS 1
#else
#error "Enable CONFIG_OMI_CODEC_OPUS in the project .conf file"
#endif

#if CODEC_OPUS
#define CODEC_PACKAGE_SAMPLES 160
#define CODEC_OUTPUT_MAX_BYTES CODEC_PACKAGE_SAMPLES * 2 // Let's assume that 16bit is enough
// P17: was OPUS_APPLICATION_RESTRICTED_LOWDELAY, which disables the SILK layer
// entirely and forces CELT-only. SILK is the half of Opus built for speech, so
// 16 kHz voice at 32 kbps was being encoded by the wrong tool -- confirmed on a
// real recording: every frame carried TOC config 22 (CELT-only WB, 10 ms).
// OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE) in codec.c was inert for the same reason.
// VOIP lets the encoder pick SILK/hybrid; at 16 kHz mono it should land on
// SILK-WB (TOC config 8), which also supports the 10 ms frames that
// CODEC_PACKAGE_SAMPLES=160 and the PC decoder both assume.
#define CODEC_OPUS_APPLICATION OPUS_APPLICATION_VOIP
#define CODEC_OPUS_BITRATE 32000
#define CODEC_OPUS_VBR 1 // Or 1
// P17: 3 -> 5 (range 0-10). Low complexity costs quality at the same bitrate.
// SILK also costs more CPU than CELT, so watch the serial log after flashing:
// `codec_ring_full` / `pcm_fail` means the codec thread can no longer keep up
// with the 10 ms frame cadence -- drop back to 3 (keeping VOIP) if so.
#define CODEC_OPUS_COMPLEXITY 5
#endif
// P17: renamed CELT -> HYBRID to match what actually ships. Neither
// CONFIG_OPUS_MODE_CELT nor CONFIG_OPUS_MODE_HYBRID is defined anywhere, so in
// codec.c both `#if` arms evaluate 0 == 0 and OPUS_ENCODER_SIZE gets defined
// twice; the second (10916, the hybrid size) wins. Verified in the P16 build
// log: `warning: "OPUS_ENCODER_SIZE" redefined ... 10916 ... 7180`. So the
// encoder buffer has always been full size and the
// `opus_encoder_get_size(1) == sizeof(m_opus_encoder)` assert in codec_start()
// still holds now that VOIP brings the SILK state into play. This line is
// documentation, not behaviour -- the value is 0 either way.
#define CONFIG_OPUS_MODE CONFIG_OPUS_MODE_HYBRID

// Codec IDs

#ifdef CODEC_OPUS
#define CODEC_ID 20
#endif

// Logs
// #define LOG_DISCARDED