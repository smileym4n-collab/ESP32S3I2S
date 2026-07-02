/*
 * ESP32-S3 native USB Audio Class speaker to external I2S DAC bridge.
 *
 * USB side:
 *   Espressif's usb_device_uac component builds the TinyUSB UAC descriptors,
 *   including the speaker streaming interface, isochronous OUT endpoint, and
 *   feedback endpoint used to keep the host paced against device buffering.
 *   This project advertises 44.1, 48, 88.2, and 96 kHz. The host-selected
 *   sample rate is reported through a callback and the I2S peripheral is
 *   reconfigured.
 *
 * I2S side:
 *   The custom board selects either a 22.5792 MHz or 24.5760 MHz oscillator
 *   with GPIO D17. The selected clock is buffered and fed to the ESP32-S3 on
 *   GPIO D15 as an external MCLK input, and to the external DAC MCLK header.
 *   The ESP32-S3 outputs BCLK, LRCK/WS, and DATA in standard Philips format.
 *   The wire format is always 32-bit stereo I2S slots so the DAC sees the
 *   conventional 64fs bit clock. 16-bit USB PCM is sign-extended into the
 *   most-significant bits of each 32-bit I2S sample word.
 *
 * Buffering:
 *   USB callbacks enqueue received PCM into a FreeRTOS stream buffer. A
 *   dedicated I2S task drains that buffer into the ESP-IDF I2S DMA driver.
 *   Playback starts only after a small prefill, underruns are padded with
 *   silence, and long idle periods stop I2S cleanly.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "usb_device_uac.h"

#define AUDIO_CHANNELS              2
#define AUDIO_DEFAULT_SAMPLE_RATE   CONFIG_AUDIO_SAMPLE_RATE
#define AUDIO_MAX_SAMPLE_RATE       96000
#define AUDIO_OSC_44K_FAMILY_HZ     22579200
#define AUDIO_OSC_48K_FAMILY_HZ     24576000
#define AUDIO_USB_BYTES_PER_SAMPLE  (CONFIG_AUDIO_BITS_PER_SAMPLE / 8)
#define AUDIO_USB_BYTES_PER_FRAME   (AUDIO_CHANNELS * AUDIO_USB_BYTES_PER_SAMPLE)
#define AUDIO_I2S_BYTES_PER_SAMPLE  4
#define AUDIO_I2S_BYTES_PER_FRAME   (AUDIO_CHANNELS * AUDIO_I2S_BYTES_PER_SAMPLE)
#define AUDIO_BUFFER_BYTES          ((AUDIO_MAX_SAMPLE_RATE * CONFIG_AUDIO_STREAM_BUFFER_MS / 1000) * AUDIO_USB_BYTES_PER_FRAME)
#define AUDIO_I2S_CHUNK_FRAMES      (CONFIG_AUDIO_I2S_DMA_FRAME_NUM / 2)
#define AUDIO_USB_CHUNK_BYTES       (AUDIO_I2S_CHUNK_FRAMES * AUDIO_USB_BYTES_PER_FRAME)
#define AUDIO_I2S_CHUNK_BYTES       (AUDIO_I2S_CHUNK_FRAMES * AUDIO_I2S_BYTES_PER_FRAME)
#define AUDIO_STATS_INTERVAL_MS     5000

#ifndef AUDIO_PIN_BCLK
#define AUDIO_PIN_BCLK              CONFIG_AUDIO_I2S_BCLK_GPIO
#endif

#ifndef AUDIO_PIN_WS
#define AUDIO_PIN_WS                CONFIG_AUDIO_I2S_WS_GPIO
#endif

#ifndef AUDIO_PIN_DATA
#define AUDIO_PIN_DATA              CONFIG_AUDIO_I2S_DATA_GPIO
#endif

#ifndef AUDIO_PIN_MCLK_IN
#define AUDIO_PIN_MCLK_IN           CONFIG_AUDIO_I2S_MCLK_GPIO
#endif

#ifndef AUDIO_PIN_OSC_SELECT
#define AUDIO_PIN_OSC_SELECT        CONFIG_AUDIO_OSC_SELECT_GPIO
#endif

#if CONFIG_IDF_TARGET_ESP32S3 && SOC_I2S_HW_VERSION_2
#define AUDIO_EXTERNAL_MCLK_SUPPORTED 1
#else
#define AUDIO_EXTERNAL_MCLK_SUPPORTED 0
#endif

#if CONFIG_UAC_SAMPLE_RATE != CONFIG_AUDIO_SAMPLE_RATE
#error "CONFIG_UAC_SAMPLE_RATE must match CONFIG_AUDIO_SAMPLE_RATE"
#endif

#if CONFIG_AUDIO_SAMPLE_RATE != 44100 && CONFIG_AUDIO_SAMPLE_RATE != 48000 && \
    CONFIG_AUDIO_SAMPLE_RATE != 88200 && CONFIG_AUDIO_SAMPLE_RATE != 96000
#error "CONFIG_AUDIO_SAMPLE_RATE must be 44100, 48000, 88200, or 96000"
#endif

#if CONFIG_UAC_SAMPLE_RATE != 44100 && CONFIG_UAC_SAMPLE_RATE != 48000 && \
    CONFIG_UAC_SAMPLE_RATE != 88200 && CONFIG_UAC_SAMPLE_RATE != 96000
#error "CONFIG_UAC_SAMPLE_RATE must be 44100, 48000, 88200, or 96000"
#endif

#if CONFIG_UAC_SPEAKER_CHANNEL_NUM != AUDIO_CHANNELS
#error "CONFIG_UAC_SPEAKER_CHANNEL_NUM must be 2 for stereo output"
#endif

#if CONFIG_UAC_MIC_CHANNEL_NUM != 0
#error "CONFIG_UAC_MIC_CHANNEL_NUM must be 0 for speaker-only firmware"
#endif

#if CONFIG_AUDIO_BITS_PER_SAMPLE != 16
#error "This firmware expects 16-bit USB PCM and outputs 32-bit Philips I2S words"
#endif

#if CONFIG_AUDIO_I2S_DMA_FRAME_NUM < 96
#error "CONFIG_AUDIO_I2S_DMA_FRAME_NUM must be at least 96 frames"
#endif

#ifndef CONFIG_AUDIO_I2S_BCLK_INVERT
#define CONFIG_AUDIO_I2S_BCLK_INVERT 0
#endif

#ifndef CONFIG_AUDIO_I2S_WS_INVERT
#define CONFIG_AUDIO_I2S_WS_INVERT 0
#endif

static const char *TAG = "usb_i2s_dac";

static i2s_chan_handle_t s_i2s_tx;
static StreamBufferHandle_t s_audio_stream;
static SemaphoreHandle_t s_i2s_lock;
static volatile bool s_i2s_running;
static volatile bool s_muted;
static volatile uint32_t s_volume = 100;
static volatile uint32_t s_underruns;
static volatile uint32_t s_overruns;
static volatile uint32_t s_short_i2s_writes;
static volatile uint32_t s_slow_i2s_writes;
static volatile uint64_t s_usb_bytes;
static volatile uint64_t s_i2s_bytes;
static volatile uint32_t s_last_i2s_write_us;
static volatile uint32_t s_max_i2s_write_us;
static volatile int64_t s_last_usb_audio_us;
static volatile uint32_t s_current_sample_rate = AUDIO_DEFAULT_SAMPLE_RATE;
static bool s_i2s_available;
#if CONFIG_AUDIO_TEST_TONE
static uint32_t s_test_tone_phase;
#endif

typedef struct {
    uint32_t sample_rate;
    uint32_t oscillator_hz;
    bool osc_select_high;
    const char *family;
} audio_clock_config_t;

static bool audio_gpio_configured(void)
{
    return AUDIO_PIN_BCLK >= 0 &&
           AUDIO_PIN_WS >= 0 &&
           AUDIO_PIN_DATA >= 0 &&
           AUDIO_PIN_MCLK_IN >= 0 &&
           AUDIO_PIN_OSC_SELECT >= 0;
}

static bool audio_clock_config_for_rate(uint32_t sample_rate, audio_clock_config_t *clock_cfg)
{
    audio_clock_config_t cfg = {
        .sample_rate = sample_rate,
    };

    switch (sample_rate) {
    case 44100:
    case 88200:
        cfg.oscillator_hz = AUDIO_OSC_44K_FAMILY_HZ;
        cfg.osc_select_high = false;
        cfg.family = "44.1 kHz";
        break;
    case 48000:
    case 96000:
        cfg.oscillator_hz = AUDIO_OSC_48K_FAMILY_HZ;
        cfg.osc_select_high = true;
        cfg.family = "48 kHz";
        break;
    default:
        return false;
    }

    if (clock_cfg != NULL) {
        *clock_cfg = cfg;
    }
    return true;
}

static bool audio_sample_rate_supported(uint32_t sample_rate)
{
    return audio_clock_config_for_rate(sample_rate, NULL);
}

static i2s_mclk_multiple_t audio_mclk_multiple(uint32_t sample_rate, uint32_t oscillator_hz)
{
    /*
     * The board sends the selected oscillator directly to the DAC as SCK/MCLK.
     * The ESP32-S3 uses that same signal as an external I2S source clock. Keep
     * the I2S peripheral's internal MCLK one divider below the external source
     * so the hardware clock tree has a real /2 source divider instead of a
     * fragile 1:1 external-MCLK path.
     */
    return (i2s_mclk_multiple_t)(oscillator_hz / sample_rate / 2);
}

static esp_err_t audio_select_oscillator(const audio_clock_config_t *clock_cfg)
{
    bool select_high = clock_cfg->osc_select_high;
#if CONFIG_AUDIO_OSC_SELECT_INVERT
    select_high = !select_high;
#endif

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << AUDIO_PIN_OSC_SELECT,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "configure oscillator select GPIO");
    ESP_RETURN_ON_ERROR(gpio_set_level(AUDIO_PIN_OSC_SELECT, select_high ? 1 : 0),
                        TAG, "set oscillator select GPIO");

    ESP_LOGI(TAG,
             "Clock select: USB rate=%" PRIu32 " Hz, oscillator family=%s, oscillator=%" PRIu32
             " Hz, D%d=%s%s",
             clock_cfg->sample_rate,
             clock_cfg->family,
             clock_cfg->oscillator_hz,
             AUDIO_PIN_OSC_SELECT,
             select_high ? "HIGH" : "LOW",
#if CONFIG_AUDIO_OSC_SELECT_INVERT
             " (inverted)"
#else
             ""
#endif
             );
    return ESP_OK;
}

static size_t audio_prefill_bytes(void)
{
    return (s_current_sample_rate * CONFIG_AUDIO_PREFILL_MS / 1000) * AUDIO_USB_BYTES_PER_FRAME;
}

static esp_err_t audio_i2s_init(uint32_t sample_rate)
{
    if (!audio_gpio_configured()) {
        ESP_LOGE(TAG,
                 "Audio pins are not configured. Set BCLK, LRCK/WS, DATA, external MCLK input, "
                 "and oscillator select GPIOs in menuconfig or build flags.");
        return ESP_ERR_INVALID_ARG;
    }

    audio_clock_config_t clock_cfg;
    if (!audio_clock_config_for_rate(sample_rate, &clock_cfg)) {
        ESP_LOGE(TAG, "Unsupported sample rate for board clocks: %" PRIu32 " Hz", sample_rate);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(audio_select_oscillator(&clock_cfg), TAG, "select oscillator");

#if !AUDIO_EXTERNAL_MCLK_SUPPORTED
    ESP_LOGE(TAG,
             "External MCLK input is not available in this ESP-IDF/I2S target configuration. "
             "This PCB expects the ESP32-S3 I2S peripheral to sync to the selected oscillator on D%d; "
             "internally generated clocks may not be valid for this design, so I2S will not start.",
             AUDIO_PIN_MCLK_IN);
    return ESP_ERR_NOT_SUPPORTED;
#else
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_AUDIO_I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = CONFIG_AUDIO_I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = CONFIG_AUDIO_I2S_DMA_FRAME_NUM;
    chan_cfg.auto_clear = true;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL), TAG, "create I2S TX channel");

    i2s_std_slot_config_t slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    slot_cfg.ws_width = 32;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = slot_cfg,
        .gpio_cfg = {
            .mclk = AUDIO_PIN_MCLK_IN,
            .bclk = AUDIO_PIN_BCLK,
            .ws = AUDIO_PIN_WS,
            .dout = AUDIO_PIN_DATA,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = CONFIG_AUDIO_I2S_BCLK_INVERT,
                .ws_inv = CONFIG_AUDIO_I2S_WS_INVERT,
            },
        },
    };

    /*
     * The selected oscillator is wired to the ESP32-S3 MCLK input and the DAC
     * MCLK header. Tell the I2S driver the actual external clock frequency so
     * BCLK and WS are derived from the same clock family as the DAC.
     */
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_EXTERNAL;
    std_cfg.clk_cfg.ext_clk_freq_hz = clock_cfg.oscillator_hz;
    std_cfg.clk_cfg.mclk_multiple = audio_mclk_multiple(sample_rate, clock_cfg.oscillator_hz);

    esp_err_t init_err = i2s_channel_init_std_mode(s_i2s_tx, &std_cfg);
    if (init_err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_del_channel(s_i2s_tx));
        s_i2s_tx = NULL;
        ESP_LOGE(TAG, "I2S std mode init failed: %s", esp_err_to_name(init_err));
        return init_err;
    }

    ESP_LOGI(TAG,
             "I2S ready: rate=%d Hz, USB bits=%d, I2S bits=32, BCLK=D%d, WS=D%d, DATA=D%d, external MCLK input=D%d, "
             "format=Philips I2S, slot_width=32, BCLK_inv=%s, WS_inv=%s, "
             "external_sck=%" PRIu32 " Hz (%" PRIu32 "fs), i2s_mclk_multiple=%d, external_mclk_used=yes",
             (int)sample_rate,
             CONFIG_AUDIO_BITS_PER_SAMPLE,
             AUDIO_PIN_BCLK,
             AUDIO_PIN_WS,
             AUDIO_PIN_DATA,
             AUDIO_PIN_MCLK_IN,
             CONFIG_AUDIO_I2S_BCLK_INVERT ? "yes" : "no",
             CONFIG_AUDIO_I2S_WS_INVERT ? "yes" : "no",
             clock_cfg.oscillator_hz,
             clock_cfg.oscillator_hz / sample_rate,
             (int)std_cfg.clk_cfg.mclk_multiple);

    return ESP_OK;
#endif
}

static void audio_i2s_deinit_locked(void)
{
    if (s_i2s_tx == NULL) {
        return;
    }

    if (s_i2s_running) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_disable(s_i2s_tx));
        s_i2s_running = false;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_del_channel(s_i2s_tx));
    s_i2s_tx = NULL;
}

static void audio_i2s_reconfigure(uint32_t sample_rate)
{
    if (!audio_sample_rate_supported(sample_rate)) {
        ESP_LOGW(TAG, "Ignoring unsupported sample rate: %" PRIu32 " Hz", sample_rate);
        return;
    }

    s_current_sample_rate = sample_rate;

    if (!s_i2s_available) {
        ESP_LOGI(TAG, "Host selected %" PRIu32 " Hz; I2S remains disabled", sample_rate);
        return;
    }

    xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
    audio_i2s_deinit_locked();
    xStreamBufferReset(s_audio_stream);
    s_last_i2s_write_us = 0;
    s_max_i2s_write_us = 0;
    esp_err_t err = audio_i2s_init(sample_rate);
    if (err != ESP_OK) {
        s_i2s_available = false;
        ESP_LOGE(TAG, "I2S reconfigure failed at %" PRIu32 " Hz: %s", sample_rate, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "I2S reconfigured for %" PRIu32 " Hz; waiting for audio prefill", sample_rate);
    }
    xSemaphoreGive(s_i2s_lock);
}

static void audio_apply_mute(uint8_t *buf, size_t len)
{
    if (s_muted || s_volume == 0) {
        memset(buf, 0, len);
    }
}

static void audio_write_i2s_sample(uint8_t **buf, int32_t sample)
{
    *(*buf)++ = (uint8_t)(sample & 0xff);
    *(*buf)++ = (uint8_t)((sample >> 8) & 0xff);
    *(*buf)++ = (uint8_t)((sample >> 16) & 0xff);
    *(*buf)++ = (uint8_t)((sample >> 24) & 0xff);
}

static int16_t audio_read_usb_sample(const uint8_t **buf)
{
    uint16_t raw = (uint16_t)(*buf)[0] | ((uint16_t)(*buf)[1] << 8);
    *buf += 2;
    return (int16_t)raw;
}

static void audio_usb_to_i2s_32(const uint8_t *usb, uint8_t *i2s, size_t frames)
{
    for (size_t frame = 0; frame < frames; frame++) {
        for (int channel = 0; channel < AUDIO_CHANNELS; channel++) {
            int32_t sample = (int32_t)audio_read_usb_sample(&usb) << 16;
            audio_write_i2s_sample(&i2s, sample);
        }
    }
}

#if CONFIG_AUDIO_TEST_TONE
static void audio_fill_test_tone(uint8_t *buf, size_t len)
{
    size_t frames = len / AUDIO_I2S_BYTES_PER_FRAME;
    uint32_t half_period_frames = s_current_sample_rate / 200;
    if (half_period_frames == 0) {
        half_period_frames = 1;
    }

    for (size_t frame = 0; frame < frames; frame++) {
        int32_t sample = ((s_test_tone_phase / half_period_frames) & 1) ? -28000 : 28000;
        sample <<= 16;
        for (int channel = 0; channel < AUDIO_CHANNELS; channel++) {
            audio_write_i2s_sample(&buf, sample);
        }

        s_test_tone_phase++;
        if (s_test_tone_phase >= half_period_frames * 2) {
            s_test_tone_phase = 0;
        }
    }
}
#endif

static void audio_i2s_task(void *arg)
{
    (void)arg;

    uint8_t usb_chunk[AUDIO_USB_CHUNK_BYTES];
    uint8_t i2s_chunk[AUDIO_I2S_CHUNK_BYTES];

    while (true) {
        if (!s_i2s_running) {
#if CONFIG_AUDIO_TEST_TONE
            xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
            if (s_i2s_tx != NULL && i2s_channel_enable(s_i2s_tx) == ESP_OK) {
                s_i2s_running = true;
                ESP_LOGW(TAG, "I2S test tone enabled at %" PRIu32 " Hz; USB audio data is bypassed",
                         s_current_sample_rate);
            }
            xSemaphoreGive(s_i2s_lock);
            if (!s_i2s_running) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
#else
            if (xStreamBufferBytesAvailable(s_audio_stream) >= audio_prefill_bytes()) {
                xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
                if (s_i2s_tx != NULL && i2s_channel_enable(s_i2s_tx) == ESP_OK) {
                    s_i2s_running = true;
                    ESP_LOGI(TAG, "I2S started at %" PRIu32 " Hz after %d ms prefill",
                             s_current_sample_rate,
                             CONFIG_AUDIO_PREFILL_MS);
                }
                xSemaphoreGive(s_i2s_lock);
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
#endif
            continue;
        }

#if CONFIG_AUDIO_TEST_TONE
        audio_fill_test_tone(i2s_chunk, sizeof(i2s_chunk));
#else
        size_t got = xStreamBufferReceive(s_audio_stream, usb_chunk, sizeof(usb_chunk), pdMS_TO_TICKS(2));
        if (got < sizeof(usb_chunk)) {
            memset(usb_chunk + got, 0, sizeof(usb_chunk) - got);
            s_underruns++;
        }
        audio_usb_to_i2s_32(usb_chunk, i2s_chunk, AUDIO_I2S_CHUNK_FRAMES);
#endif

#if !CONFIG_AUDIO_TEST_TONE
        audio_apply_mute(i2s_chunk, sizeof(i2s_chunk));
#endif

        size_t bytes_written = 0;
        xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (s_i2s_tx != NULL && s_i2s_running) {
            int64_t write_start_us = esp_timer_get_time();
            err = i2s_channel_write(
                s_i2s_tx,
                i2s_chunk,
                sizeof(i2s_chunk),
                &bytes_written,
                pdMS_TO_TICKS(20));
            uint32_t write_us = (uint32_t)(esp_timer_get_time() - write_start_us);
            s_last_i2s_write_us = write_us;
            if (write_us > s_max_i2s_write_us) {
                s_max_i2s_write_us = write_us;
            }
            if (write_us > 5000) {
                s_slow_i2s_writes++;
            }
        }
        xSemaphoreGive(s_i2s_lock);

        if (err != ESP_OK || bytes_written != sizeof(i2s_chunk)) {
            s_short_i2s_writes++;
            ESP_LOGW(TAG,
                     "I2S short write: err=%s written=%u requested=%u",
                     esp_err_to_name(err),
                     (unsigned)bytes_written,
                     (unsigned)sizeof(i2s_chunk));
        } else {
            s_i2s_bytes += bytes_written;
        }

#if !CONFIG_AUDIO_TEST_TONE
        const int64_t idle_ms = (esp_timer_get_time() - s_last_usb_audio_us) / 1000;
        if (idle_ms > CONFIG_AUDIO_IDLE_STOP_MS) {
            xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
            if (s_i2s_tx != NULL) {
                ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_disable(s_i2s_tx));
            }
            xStreamBufferReset(s_audio_stream);
            s_i2s_running = false;
            xSemaphoreGive(s_i2s_lock);
            ESP_LOGI(TAG, "I2S stopped after %" PRId64 " ms without USB audio", idle_ms);
        }
#endif
    }
}

static void audio_stats_task(void *arg)
{
    (void)arg;

    uint32_t last_underruns = 0;
    uint32_t last_overruns = 0;
    uint32_t last_short_writes = 0;
    uint32_t last_slow_writes = 0;
    uint64_t last_usb_bytes = 0;
    uint64_t last_i2s_bytes = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(AUDIO_STATS_INTERVAL_MS));

        uint32_t underruns = s_underruns;
        uint32_t overruns = s_overruns;
        uint32_t short_writes = s_short_i2s_writes;
        uint32_t slow_writes = s_slow_i2s_writes;
        uint64_t usb_bytes = s_usb_bytes;
        uint64_t i2s_bytes = s_i2s_bytes;
        uint32_t usb_bytes_per_s = (uint32_t)((usb_bytes - last_usb_bytes) * 1000 / AUDIO_STATS_INTERVAL_MS);
        uint32_t i2s_bytes_per_s = (uint32_t)((i2s_bytes - last_i2s_bytes) * 1000 / AUDIO_STATS_INTERVAL_MS);

        if (underruns != last_underruns ||
            overruns != last_overruns ||
            short_writes != last_short_writes ||
            slow_writes != last_slow_writes ||
            usb_bytes != last_usb_bytes ||
            i2s_bytes != last_i2s_bytes) {
            ESP_LOGW(TAG,
                     "audio stats: underruns=%" PRIu32 " overruns=%" PRIu32
                     " short_i2s_writes=%" PRIu32 " slow_i2s_writes=%" PRIu32
                     " buffered=%u bytes usb=%" PRIu32 " B/s i2s=%" PRIu32
                     " B/s last_write=%" PRIu32 " us max_write=%" PRIu32 " us i2s=%s",
                     underruns,
                     overruns,
                     short_writes,
                     slow_writes,
                     (unsigned)xStreamBufferBytesAvailable(s_audio_stream),
                     usb_bytes_per_s,
                     i2s_bytes_per_s,
                     s_last_i2s_write_us,
                     s_max_i2s_write_us,
                     s_i2s_running ? "running" : "stopped");
            last_underruns = underruns;
            last_overruns = overruns;
            last_short_writes = short_writes;
            last_slow_writes = slow_writes;
            last_usb_bytes = usb_bytes;
            last_i2s_bytes = i2s_bytes;
        }
    }
}

static void audio_pin_wait_task(void *arg)
{
    (void)arg;

    while (true) {
        ESP_LOGE(TAG,
                 "I2S is disabled. Check the previous log for pin configuration or external MCLK support errors.");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *cb_ctx)
{
    (void)cb_ctx;

    if (!s_i2s_available) {
        return ESP_OK;
    }

    s_last_usb_audio_us = esp_timer_get_time();

    size_t written = xStreamBufferSend(s_audio_stream, buf, len, pdMS_TO_TICKS(2));
    if (written != len) {
        s_overruns++;
    }
    s_usb_bytes += written;

    return ESP_OK;
}

static void uac_device_set_mute_cb(uint32_t mute, void *cb_ctx)
{
    (void)cb_ctx;
    s_muted = mute != 0;
    ESP_LOGI(TAG, "USB mute set to %" PRIu32, mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *cb_ctx)
{
    (void)cb_ctx;
    s_volume = volume;
    ESP_LOGI(TAG, "USB volume set to %" PRIu32, volume);
}

static void uac_device_set_sample_rate_cb(uint32_t sample_rate, void *cb_ctx)
{
    (void)cb_ctx;
    ESP_LOGI(TAG, "USB host selected sample rate: %" PRIu32 " Hz", sample_rate);
    audio_i2s_reconfigure(sample_rate);
}

void app_main(void)
{
    ESP_LOGI(TAG,
             "Starting USB Audio speaker: 44.1/48/88.2/96 kHz, %d-bit stereo, Philips I2S, "
             "external MCLK input on D%d, oscillator select on D%d",
             CONFIG_AUDIO_BITS_PER_SAMPLE,
             AUDIO_PIN_MCLK_IN,
             AUDIO_PIN_OSC_SELECT);
    ESP_LOGI(TAG,
             "Audio frame sizes: USB=%d bytes/frame, I2S=%d bytes/frame; at 48 kHz expect usb=192000 B/s i2s=384000 B/s",
             AUDIO_USB_BYTES_PER_FRAME,
             AUDIO_I2S_BYTES_PER_FRAME);
#if CONFIG_AUDIO_TEST_TONE
    ESP_LOGW(TAG, "Audio test tone build is enabled; USB audio data will not be played");
#endif

#if !AUDIO_EXTERNAL_MCLK_SUPPORTED
    ESP_LOGE(TAG,
             "This build target does not expose ESP-IDF I2S external MCLK input support. "
             "USB may enumerate for descriptor testing, but I2S playback is disabled.");
#endif

    s_i2s_lock = xSemaphoreCreateMutex();
    if (s_i2s_lock == NULL) {
        ESP_LOGE(TAG, "failed to create I2S mutex");
        abort();
    }

    esp_err_t i2s_err = audio_i2s_init(s_current_sample_rate);
    if (i2s_err == ESP_OK) {
        s_i2s_available = true;
    } else {
        ESP_LOGE(TAG,
                 "I2S disabled; USB can still enumerate for descriptor testing");
    }

    s_audio_stream = xStreamBufferCreate(AUDIO_BUFFER_BYTES, AUDIO_USB_BYTES_PER_FRAME);
    if (s_audio_stream == NULL) {
        ESP_LOGE(TAG, "failed to create %u-byte audio stream buffer", (unsigned)AUDIO_BUFFER_BYTES);
        abort();
    }

    s_last_usb_audio_us = esp_timer_get_time();

    BaseType_t task_ok = pdPASS;
    if (s_i2s_available) {
        task_ok = xTaskCreatePinnedToCore(
            audio_i2s_task,
            "audio_i2s",
            4096,
            NULL,
            8,
            NULL,
            tskNO_AFFINITY);
    } else {
        task_ok = xTaskCreatePinnedToCore(
            audio_pin_wait_task,
            "audio_pin_wait",
            3072,
            NULL,
            2,
            NULL,
            tskNO_AFFINITY);
    }
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create audio_i2s task");
        abort();
    }

    task_ok = xTaskCreatePinnedToCore(
        audio_stats_task,
        "audio_stats",
        3072,
        NULL,
        3,
        NULL,
        tskNO_AFFINITY);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create audio_stats task");
        abort();
    }

    uac_device_config_t uac_config = {
        .skip_tinyusb_init = false,
        .output_cb = uac_device_output_cb,
        .input_cb = NULL,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .set_sample_rate_cb = uac_device_set_sample_rate_cb,
        .cb_ctx = NULL,
    };

    ESP_ERROR_CHECK(uac_device_init(&uac_config));
    ESP_LOGI(TAG, "USB UAC initialized; waiting for host");
}
