/*
 * ESP32-S3 native USB Audio Class speaker to PCM1798 I2S bridge.
 *
 * USB side:
 *   Espressif's usb_device_uac component builds the TinyUSB UAC descriptors,
 *   including the speaker streaming interface, isochronous OUT endpoint, and
 *   feedback endpoint used to keep the host paced against device buffering.
 *   The component currently advertises one compile-time sample rate, so this
 *   project keeps 48 kHz and 96 kHz as mutually exclusive sdkconfig builds.
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
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "usb_device_uac.h"

#define AUDIO_CHANNELS              2
#define AUDIO_BYTES_PER_SAMPLE      (CONFIG_AUDIO_BITS_PER_SAMPLE / 8)
#define AUDIO_BYTES_PER_FRAME       (AUDIO_CHANNELS * AUDIO_BYTES_PER_SAMPLE)
#define AUDIO_BUFFER_BYTES          ((CONFIG_AUDIO_SAMPLE_RATE * CONFIG_AUDIO_STREAM_BUFFER_MS / 1000) * AUDIO_BYTES_PER_FRAME)
#define AUDIO_PREFILL_BYTES         ((CONFIG_AUDIO_SAMPLE_RATE * CONFIG_AUDIO_PREFILL_MS / 1000) * AUDIO_BYTES_PER_FRAME)
#define AUDIO_I2S_CHUNK_FRAMES      CONFIG_AUDIO_I2S_DMA_FRAME_NUM
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

static const char *TAG = "usb_i2s_dac";

static i2s_chan_handle_t s_i2s_tx;
static StreamBufferHandle_t s_audio_stream;
static volatile bool s_usb_mounted;
static volatile bool s_i2s_running;
static volatile bool s_muted;
static volatile uint32_t s_volume = 100;
static volatile uint32_t s_underruns;
static volatile uint32_t s_overruns;
static volatile uint32_t s_short_i2s_writes;
static volatile int64_t s_last_usb_audio_us;

void tud_mount_cb(void)
{
    s_usb_mounted = true;
    ESP_LOGI(TAG, "USB audio device mounted by host");
}

void tud_umount_cb(void)
{
    s_usb_mounted = false;
    ESP_LOGI(TAG, "USB audio device unmounted/disconnected");
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    ESP_LOGI(TAG, "USB bus suspended");
}

void tud_resume_cb(void)
{
    ESP_LOGI(TAG, "USB bus resumed");
}

static bool audio_gpio_configured(void)
{
    return AUDIO_PIN_BCLK >= 0 &&
           AUDIO_PIN_WS >= 0 &&
           AUDIO_PIN_DATA >= 0;
}

static esp_err_t audio_i2s_init(void)
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
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_AUDIO_SAMPLE_RATE),
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
        (CONFIG_AUDIO_SAMPLE_RATE == 96000) ? I2S_MCLK_MULTIPLE_192 : I2S_MCLK_MULTIPLE_384;
#else
    std_cfg.clk_cfg.mclk_multiple =
        (CONFIG_AUDIO_SAMPLE_RATE == 96000) ? I2S_MCLK_MULTIPLE_128 : I2S_MCLK_MULTIPLE_256;
#endif

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg), TAG, "init I2S std mode");

    ESP_LOGI(TAG,
             "I2S ready: rate=%d Hz, bits=%d, BCLK=%d, WS=%d, DATA=%d, MCLK pin=unused",
             CONFIG_AUDIO_SAMPLE_RATE,
             CONFIG_AUDIO_BITS_PER_SAMPLE,
             AUDIO_PIN_BCLK,
             AUDIO_PIN_WS,
             AUDIO_PIN_DATA);

    return ESP_OK;
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
            if (xStreamBufferBytesAvailable(s_audio_stream) >= AUDIO_PREFILL_BYTES) {
                if (i2s_channel_enable(s_i2s_tx) == ESP_OK) {
                    s_i2s_running = true;
                    ESP_LOGI(TAG, "I2S started after %d ms prefill", CONFIG_AUDIO_PREFILL_MS);
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            continue;
        }

        size_t got = xStreamBufferReceive(s_audio_stream, chunk, sizeof(chunk), pdMS_TO_TICKS(5));
        if (got < sizeof(chunk)) {
            memset(chunk + got, 0, sizeof(chunk) - got);
            s_underruns++;
        }

        audio_apply_mute(chunk, sizeof(chunk));

        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(
            s_i2s_tx,
            chunk,
            sizeof(chunk),
            &bytes_written,
            pdMS_TO_TICKS(20));

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
            ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_disable(s_i2s_tx));
            xStreamBufferReset(s_audio_stream);
            s_i2s_running = false;
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
                     " short_i2s_writes=%" PRIu32 " buffered=%u bytes usb=%s i2s=%s",
                     underruns,
                     overruns,
                     short_writes,
                     (unsigned)xStreamBufferBytesAvailable(s_audio_stream),
                     s_usb_mounted ? "mounted" : "not-mounted",
                     s_i2s_running ? "running" : "stopped");
            last_underruns = underruns;
            last_overruns = overruns;
            last_short_writes = short_writes;
        }
    }
}

static esp_err_t uac_device_output_cb(uint8_t *buf, size_t len, void *cb_ctx)
{
    (void)cb_ctx;

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

void app_main(void)
{
    ESP_LOGI(TAG,
             "Starting USB Audio speaker: %d Hz, %d-bit stereo, Philips I2S, no ESP MCLK output",
             CONFIG_AUDIO_SAMPLE_RATE,
             CONFIG_AUDIO_BITS_PER_SAMPLE);

    ESP_ERROR_CHECK(audio_i2s_init());

    s_audio_stream = xStreamBufferCreate(AUDIO_BUFFER_BYTES, AUDIO_BYTES_PER_FRAME);
    if (s_audio_stream == NULL) {
        ESP_LOGE(TAG, "failed to create %u-byte audio stream buffer", (unsigned)AUDIO_BUFFER_BYTES);
        abort();
    }

    s_last_usb_audio_us = esp_timer_get_time();

    BaseType_t task_ok = xTaskCreatePinnedToCore(
        audio_i2s_task,
        "audio_i2s",
        4096,
        NULL,
        8,
        NULL,
        tskNO_AFFINITY);
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
        .cb_ctx = NULL,
    };

    ESP_ERROR_CHECK(uac_device_init(&uac_config));
    ESP_LOGI(TAG, "USB UAC initialized; waiting for host");
}
