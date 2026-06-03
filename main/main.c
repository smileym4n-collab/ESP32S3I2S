/*
 * ESP32-S3 native USB Audio Class speaker to PCM1798 I2S bridge.
 *
 * USB side:
 *   Espressif's usb_device_uac component builds the TinyUSB UAC descriptors,
 *   including the speaker streaming interface, isochronous OUT endpoint, and
 *   feedback endpoint used to keep the host paced against device buffering.
 *   This project advertises 48 kHz and 96 kHz. The host-selected sample rate
 *   is reported through a callback and the I2S peripheral is reconfigured.
 *
 * I2S side:
 *   The custom board feeds the DAC MCLK pin from a fixed 12.288 MHz oscillator.
 *   The ESP32-S3 therefore leaves MCLK unassigned and outputs only BCLK,
 *   LRCK/WS, and DATA in standard Philips format. I2S slots are widened to
 *   32 bits so the DAC sees the conventional 64fs bit clock even for 16-bit
 *   USB PCM.
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
#define AUDIO_DEFAULT_SAMPLE_RATE   48000
#define AUDIO_MAX_SAMPLE_RATE       96000
#define AUDIO_BYTES_PER_SAMPLE      (CONFIG_AUDIO_BITS_PER_SAMPLE / 8)
#define AUDIO_BYTES_PER_FRAME       (AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)
#define AUDIO_BUFFER_BYTES          ((AUDIO_MAX_SAMPLE_RATE * CONFIG_AUDIO_STREAM_BUFFER_MS / 1000) * AUDIO_BYTES_PER_FRAME)
#define AUDIO_I2S_CHUNK_FRAMES      (CONFIG_AUDIO_I2S_DMA_FRAME_NUM / 2)
#define AUDIO_I2S_CHUNK_BYTES       (AUDIO_I2S_CHUNK_FRAMES * AUDIO_BYTES_PER_FRAME)
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

#if CONFIG_UAC_SAMPLE_RATE != CONFIG_AUDIO_SAMPLE_RATE
#error "CONFIG_UAC_SAMPLE_RATE must match CONFIG_AUDIO_SAMPLE_RATE"
#endif

#if CONFIG_UAC_SPEAKER_CHANNEL_NUM != AUDIO_CHANNELS
#error "CONFIG_UAC_SPEAKER_CHANNEL_NUM must be 2 for stereo output"
#endif

#if CONFIG_UAC_MIC_CHANNEL_NUM != 0
#error "CONFIG_UAC_MIC_CHANNEL_NUM must be 0 for speaker-only firmware"
#endif

#if CONFIG_AUDIO_BITS_PER_SAMPLE != 16 && CONFIG_AUDIO_BITS_PER_SAMPLE != 24
#error "Only 16-bit and 24-bit stereo PCM are supported"
#endif

#if CONFIG_AUDIO_BITS_PER_SAMPLE == 24
#warning "Verify the USB UAC speaker descriptors advertise 24-bit PCM before using this build"
#endif

#if CONFIG_AUDIO_BITS_PER_SAMPLE == 24 && (CONFIG_AUDIO_I2S_DMA_FRAME_NUM % 3) != 0
#error "For 24-bit I2S, CONFIG_AUDIO_I2S_DMA_FRAME_NUM must be a multiple of 3"
#endif

#if CONFIG_AUDIO_I2S_DMA_FRAME_NUM < 96
#error "CONFIG_AUDIO_I2S_DMA_FRAME_NUM must be at least 96 frames"
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
static volatile int64_t s_last_usb_audio_us;
static volatile uint32_t s_current_sample_rate = AUDIO_DEFAULT_SAMPLE_RATE;
static bool s_i2s_available;

static bool audio_gpio_configured(void)
{
    return AUDIO_PIN_BCLK >= 0 &&
           AUDIO_PIN_WS >= 0 &&
           AUDIO_PIN_DATA >= 0;
}

static bool audio_sample_rate_supported(uint32_t sample_rate)
{
    return sample_rate == 48000 || sample_rate == 96000;
}

static size_t audio_prefill_bytes(void)
{
    return (s_current_sample_rate * CONFIG_AUDIO_PREFILL_MS / 1000) * AUDIO_BYTES_PER_FRAME;
}

static esp_err_t audio_i2s_init(uint32_t sample_rate)
{
    if (!audio_gpio_configured()) {
        ESP_LOGE(TAG,
                 "I2S pins are not configured. Set BCLK, LRCK/WS, and DATA in menuconfig.");
        return ESP_ERR_INVALID_ARG;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_AUDIO_I2S_PORT, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = CONFIG_AUDIO_I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = CONFIG_AUDIO_I2S_DMA_FRAME_NUM;
    chan_cfg.auto_clear = true;

    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx, NULL), TAG, "create I2S TX channel");

    i2s_data_bit_width_t data_width = I2S_DATA_BIT_WIDTH_16BIT;
#if CONFIG_AUDIO_BITS_PER_SAMPLE == 24
    data_width = I2S_DATA_BIT_WIDTH_24BIT;
#endif

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(data_width, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AUDIO_PIN_BCLK,
            .ws = AUDIO_PIN_WS,
            .dout = AUDIO_PIN_DATA,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    /*
     * PCM1798-class DACs normally expect a 64fs BCLK in Philips I2S mode.
     * Keeping 32-bit slots gives 3.072 MHz BCLK at 48 kHz and 6.144 MHz at
     * 96 kHz. The ESP32-S3 still transmits only the configured sample width.
     */
    std_cfg.slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
    std_cfg.slot_cfg.ws_width = 32;

    /*
     * This clock exists inside the I2S peripheral even though the MCLK pin is
     * unused. For the 16-bit default, 256fs at 48 kHz and 128fs at 96 kHz
     * mirror the DAC's external oscillator ratios. ESP-IDF recommends an
     * MCLK multiple divisible by 3 for 24-bit I2S data, so the experimental
     * 24-bit path uses 384fs/192fs internally while still not driving MCLK.
     */
#if CONFIG_AUDIO_BITS_PER_SAMPLE == 24
    std_cfg.clk_cfg.mclk_multiple =
        (sample_rate == 96000) ? I2S_MCLK_MULTIPLE_192 : I2S_MCLK_MULTIPLE_384;
#else
    std_cfg.clk_cfg.mclk_multiple =
        (sample_rate == 96000) ? I2S_MCLK_MULTIPLE_128 : I2S_MCLK_MULTIPLE_256;
#endif

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg), TAG, "init I2S std mode");

    ESP_LOGI(TAG,
             "I2S ready: rate=%d Hz, bits=%d, BCLK=%d, WS=%d, DATA=%d, MCLK pin=unused",
             (int)sample_rate,
             CONFIG_AUDIO_BITS_PER_SAMPLE,
             AUDIO_PIN_BCLK,
             AUDIO_PIN_WS,
             AUDIO_PIN_DATA);

    return ESP_OK;
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
        ESP_LOGI(TAG, "Host selected %" PRIu32 " Hz; I2S remains disabled until pins are configured", sample_rate);
        return;
    }

    xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
    audio_i2s_deinit_locked();
    xStreamBufferReset(s_audio_stream);
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

static void audio_i2s_task(void *arg)
{
    (void)arg;

    uint8_t chunk[AUDIO_I2S_CHUNK_BYTES];

    while (true) {
        if (!s_i2s_running) {
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
            continue;
        }

        size_t got = xStreamBufferReceive(s_audio_stream, chunk, sizeof(chunk), pdMS_TO_TICKS(2));
        if (got < sizeof(chunk)) {
            memset(chunk + got, 0, sizeof(chunk) - got);
            s_underruns++;
        }

        audio_apply_mute(chunk, sizeof(chunk));

        size_t bytes_written = 0;
        xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
        esp_err_t err = ESP_ERR_INVALID_STATE;
        if (s_i2s_tx != NULL) {
            err = i2s_channel_write(
                s_i2s_tx,
                chunk,
                sizeof(chunk),
                &bytes_written,
                pdMS_TO_TICKS(20));
        }
        xSemaphoreGive(s_i2s_lock);

        if (err != ESP_OK || bytes_written != sizeof(chunk)) {
            s_short_i2s_writes++;
            ESP_LOGW(TAG,
                     "I2S short write: err=%s written=%u requested=%u",
                     esp_err_to_name(err),
                     (unsigned)bytes_written,
                     (unsigned)sizeof(chunk));
        }

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
    }
}

static void audio_stats_task(void *arg)
{
    (void)arg;

    uint32_t last_underruns = 0;
    uint32_t last_overruns = 0;
    uint32_t last_short_writes = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(AUDIO_STATS_INTERVAL_MS));

        uint32_t underruns = s_underruns;
        uint32_t overruns = s_overruns;
        uint32_t short_writes = s_short_i2s_writes;

        if (underruns != last_underruns ||
            overruns != last_overruns ||
            short_writes != last_short_writes) {
            ESP_LOGW(TAG,
                     "audio stats: underruns=%" PRIu32 " overruns=%" PRIu32
                     " short_i2s_writes=%" PRIu32 " buffered=%u bytes i2s=%s",
                     underruns,
                     overruns,
                     short_writes,
                     (unsigned)xStreamBufferBytesAvailable(s_audio_stream),
                     s_i2s_running ? "running" : "stopped");
            last_underruns = underruns;
            last_overruns = overruns;
            last_short_writes = short_writes;
        }
    }
}

static void audio_pin_wait_task(void *arg)
{
    (void)arg;

    while (true) {
        ESP_LOGE(TAG,
                 "I2S pins are not configured. Edit platformio.ini and set "
                 "AUDIO_PIN_BCLK, AUDIO_PIN_WS, and AUDIO_PIN_DATA to real positive GPIO numbers.");
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
             "Starting USB Audio speaker: 48/96 kHz, %d-bit stereo, Philips I2S, no ESP MCLK output",
             CONFIG_AUDIO_BITS_PER_SAMPLE);

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
                 "I2S disabled until pins are configured; USB can still enumerate for descriptor testing");
    }

    s_audio_stream = xStreamBufferCreate(AUDIO_BUFFER_BYTES, AUDIO_BYTES_PER_FRAME);
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
