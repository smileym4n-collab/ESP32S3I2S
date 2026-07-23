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
 *   with GPIO7. The selected clock is buffered and fed to the ESP32-S3 on
 *   GPIO15 as an external MCLK input, and to the external DAC MCLK header.
 *   The ESP32-S3 outputs BCLK, LRCK/WS, and DATA in standard Philips format.
 *   The wire format is always 32-bit stereo I2S slots so the DAC sees the
 *   conventional 64fs bit clock. 16-bit and 24-bit USB PCM are aligned into
 *   the most-significant bits of each 32-bit I2S sample word.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"
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
#define AUDIO_USB_MAX_BYTES_PER_SAMPLE  4
#define AUDIO_USB_MAX_BYTES_PER_FRAME   (AUDIO_CHANNELS * AUDIO_USB_MAX_BYTES_PER_SAMPLE)
#define AUDIO_I2S_BYTES_PER_SAMPLE  4
#define AUDIO_I2S_BYTES_PER_FRAME   (AUDIO_CHANNELS * AUDIO_I2S_BYTES_PER_SAMPLE)
#define AUDIO_BUFFER_BYTES          ((AUDIO_MAX_SAMPLE_RATE * CONFIG_AUDIO_STREAM_BUFFER_MS / 1000) * AUDIO_USB_MAX_BYTES_PER_FRAME)
#define AUDIO_I2S_CHUNK_FRAMES      (CONFIG_AUDIO_I2S_DMA_FRAME_NUM / 2)
#define AUDIO_USB_CHUNK_BYTES       (AUDIO_I2S_CHUNK_FRAMES * AUDIO_USB_MAX_BYTES_PER_FRAME)
#define AUDIO_I2S_CHUNK_BYTES       (AUDIO_I2S_CHUNK_FRAMES * AUDIO_I2S_BYTES_PER_FRAME)
#define AUDIO_STATS_INTERVAL_MS     5000
#define AUDIO_SETTLING_DELAY_MS     500
#define ATTINY_UART_PORT            UART_NUM_1
#define ATTINY_UART_BAUD            9600

#define PIN_I2C_SDA                 CONFIG_BOARD_I2C_SDA_GPIO
#define PIN_I2C_SCL                 CONFIG_BOARD_I2C_SCL_GPIO
#define PIN_ATTINY_UART_TX          CONFIG_BOARD_ATTINY_UART_TX_GPIO
#define PIN_VBUS_SENSE              CONFIG_BOARD_VBUS_SENSE_GPIO
#define PIN_MUTE                    CONFIG_BOARD_MUTE_GPIO
#define PIN_RATE_F3                 CONFIG_BOARD_RATE_F3_GPIO
#define PIN_RATE_F2                 CONFIG_BOARD_RATE_F2_GPIO
#define PIN_RATE_F1                 CONFIG_BOARD_RATE_F1_GPIO
#define PIN_RATE_F0                 CONFIG_BOARD_RATE_F0_GPIO

#ifndef BOARD_STATUS_OUTPUTS_ENABLE
#define BOARD_STATUS_OUTPUTS_ENABLE 1
#endif

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

#if CONFIG_AUDIO_I2S_DMA_FRAME_NUM < 96
#error "CONFIG_AUDIO_I2S_DMA_FRAME_NUM must be at least 96 frames"
#endif

#ifndef CONFIG_AUDIO_I2S_BCLK_INVERT
#define CONFIG_AUDIO_I2S_BCLK_INVERT 0
#endif

#ifndef CONFIG_AUDIO_I2S_WS_INVERT
#define CONFIG_AUDIO_I2S_WS_INVERT 0
#endif

#if BOARD_STATUS_OUTPUTS_ENABLE && CONFIG_SPIRAM_MODE_OCT && \
    (CONFIG_BOARD_MUTE_GPIO >= 33 && CONFIG_BOARD_MUTE_GPIO <= 37 || \
     CONFIG_BOARD_RATE_F3_GPIO >= 33 && CONFIG_BOARD_RATE_F3_GPIO <= 37 || \
     CONFIG_BOARD_RATE_F2_GPIO >= 33 && CONFIG_BOARD_RATE_F2_GPIO <= 37 || \
     CONFIG_BOARD_RATE_F1_GPIO >= 33 && CONFIG_BOARD_RATE_F1_GPIO <= 37 || \
     CONFIG_BOARD_RATE_F0_GPIO >= 33 && CONFIG_BOARD_RATE_F0_GPIO <= 37)
#define BOARD_OCTAL_PSRAM_PIN_CONFLICT 1
#pragma message("WARNING: GPIO33-37 conflict with octal PSRAM; R8 modules cannot use the requested MUTE/F pinout")
#else
#define BOARD_OCTAL_PSRAM_PIN_CONFLICT 0
#endif

#if !BOARD_STATUS_OUTPUTS_ENABLE
#pragma message("N8R8 AUDIO TEST: GPIO35-39 MUTE/F outputs are disabled and will not be configured")
#endif

static const char *TAG = "usb_i2s_dac";

static i2s_chan_handle_t s_i2s_tx;
static StreamBufferHandle_t s_audio_stream;
static SemaphoreHandle_t s_i2s_lock;
static volatile bool s_i2s_running;
static volatile bool s_usb_muted;
static volatile bool s_hw_muted = true;
static volatile bool s_usb_connected;
static volatile bool s_stream_active;
static volatile bool s_vbus_present;
static volatile int64_t s_settle_until_us;
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
static volatile uint32_t s_current_bits_per_sample = 16;
static volatile uint32_t s_current_usb_bytes_per_sample = 2;
static volatile uint32_t s_current_usb_bytes_per_frame = AUDIO_CHANNELS * 2;
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

static bool audio_sample_rate_supported(uint32_t sample_rate);

static void status_uart_send(const char *message)
{
    if (message == NULL) {
        return;
    }
    uart_write_bytes(ATTINY_UART_PORT, message, strlen(message));
    uart_write_bytes(ATTINY_UART_PORT, "\r\n", 2);
    ESP_LOGI(TAG, "ATtiny UART TX: %s", message);
}

static void status_uart_send_state(void)
{
    char line[80];
    if (!s_usb_connected || !s_vbus_present) {
        snprintf(line, sizeof(line), "USB:0,MUTE:1");
    } else {
        snprintf(line,
                 sizeof(line),
                 "SR:%" PRIu32 ",BITS:%" PRIu32 ",MUTE:%d,USB:1",
                 s_current_sample_rate,
                 s_current_bits_per_sample,
                 s_hw_muted ? 1 : 0);
    }
    status_uart_send(line);
}

static void audio_set_rate_outputs(bool active)
{
    uint8_t code = 0;
    if (active) {
        switch (s_current_sample_rate) {
        case 44100: code = 0x1; break;
        case 48000: code = 0x2; break;
        case 88200: code = 0x3; break;
        case 96000: code = 0x4; break;
        default: active = false; break;
        }
    }

#if BOARD_STATUS_OUTPUTS_ENABLE
    gpio_set_level(PIN_RATE_F3, (code >> 3) & 1);
    gpio_set_level(PIN_RATE_F2, (code >> 2) & 1);
    gpio_set_level(PIN_RATE_F1, (code >> 1) & 1);
    gpio_set_level(PIN_RATE_F0, code & 1);
#endif
    ESP_LOGI(TAG,
             "Sample-rate status: rate=%" PRIu32 " F3F2F1F0=%d%d%d%d (%s, GPIO outputs %s)",
             s_current_sample_rate,
             (code >> 3) & 1,
             (code >> 2) & 1,
             (code >> 1) & 1,
             code & 1,
             active ? "active" : "inactive",
             BOARD_STATUS_OUTPUTS_ENABLE ? "enabled" : "disabled");
}

static void audio_set_hw_mute(bool muted, const char *reason)
{
#if BOARD_STATUS_OUTPUTS_ENABLE
    gpio_set_level(PIN_MUTE, muted ? 1 : 0);
#endif
    audio_set_rate_outputs(!muted);
    if (s_hw_muted != muted) {
        s_hw_muted = muted;
        ESP_LOGI(TAG,
                 "MUTE %s=%s (%s)",
                 BOARD_STATUS_OUTPUTS_ENABLE ? "GPIO35" : "logical state; GPIO disabled",
                 muted ? "HIGH" : "LOW",
                 reason);
        status_uart_send_state();
    }
}

static bool audio_can_run(void)
{
    return s_vbus_present &&
           s_usb_connected &&
           s_stream_active &&
           s_i2s_available &&
           audio_sample_rate_supported(s_current_sample_rate);
}

static void board_status_io_init(void)
{
#if BOARD_OCTAL_PSRAM_PIN_CONFLICT
    ESP_LOGE(TAG,
             "BOARD/MODULE CONFLICT: the requested MUTE/F pins include GPIO35-37, "
             "which Espressif reserves for octal PSRAM on R8 modules. "
             "Use a non-octal WROOM-1U variant or revise the PCB pinout.");
#endif

#if BOARD_STATUS_OUTPUTS_ENABLE
    gpio_config_t output_cfg = {
        .pin_bit_mask = (1ULL << PIN_MUTE) |
                        (1ULL << PIN_RATE_F3) |
                        (1ULL << PIN_RATE_F2) |
                        (1ULL << PIN_RATE_F1) |
                        (1ULL << PIN_RATE_F0),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_cfg));
    gpio_set_level(PIN_MUTE, 1);
    audio_set_rate_outputs(false);
#else
    ESP_LOGW(TAG,
             "N8R8 audio-test build: MUTE and F0-F3 GPIO outputs are disabled; "
             "GPIO35-39 will not be configured");
#endif

    gpio_config_t vbus_cfg = {
        .pin_bit_mask = 1ULL << PIN_VBUS_SENSE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&vbus_cfg));
    s_vbus_present = gpio_get_level(PIN_VBUS_SENSE) != 0;
    ESP_LOGI(TAG,
             "VBUS sense GPIO%d: %s (external resistor divider input)",
             PIN_VBUS_SENSE,
             s_vbus_present ? "present" : "missing");
    ESP_LOGI(TAG,
             "I2C reserved (not initialized): SDA=GPIO%d SCL=GPIO%d",
             PIN_I2C_SDA,
             PIN_I2C_SCL);

    uart_config_t uart_cfg = {
        .baud_rate = ATTINY_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(ATTINY_UART_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(ATTINY_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(ATTINY_UART_PORT,
                                 PIN_ATTINY_UART_TX,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG,
             "ATtiny status UART: UART%d TX=GPIO%d, %d 8N1; UART0 remains reserved for console",
             ATTINY_UART_PORT,
             PIN_ATTINY_UART_TX,
             ATTINY_UART_BAUD);
}

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
             " Hz, GPIO%d=%s%s",
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
    return (s_current_sample_rate * CONFIG_AUDIO_PREFILL_MS / 1000) * s_current_usb_bytes_per_frame;
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
             "This PCB expects the ESP32-S3 I2S peripheral to sync to the selected oscillator on GPIO%d; "
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
             "I2S ready: rate=%d Hz, USB bits=%d, I2S bits=32, BCLK=GPIO%d, WS=GPIO%d, DATA=GPIO%d, external MCLK input=GPIO%d, "
             "format=Philips I2S, slot_width=32, BCLK_inv=%s, WS_inv=%s, "
             "external_sck=%" PRIu32 " Hz (%" PRIu32 "fs), i2s_mclk_multiple=%d, external_mclk_used=yes",
             (int)sample_rate,
             (int)s_current_bits_per_sample,
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

static void audio_stop(const char *reason)
{
    audio_set_hw_mute(true, reason);
    if (s_i2s_lock == NULL) {
        return;
    }
    xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
    if (s_i2s_tx != NULL && s_i2s_running) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_disable(s_i2s_tx));
    }
    s_i2s_running = false;
    if (s_audio_stream != NULL) {
        xStreamBufferReset(s_audio_stream);
    }
    xSemaphoreGive(s_i2s_lock);
    ESP_LOGI(TAG, "Audio stopped: %s", reason);
}

static void audio_i2s_reconfigure(uint32_t sample_rate)
{
    audio_set_hw_mute(true, "sample-rate change");
    if (!audio_sample_rate_supported(sample_rate)) {
        ESP_LOGE(TAG, "Unsupported sample rate: %" PRIu32 " Hz", sample_rate);
        audio_set_rate_outputs(false);
        status_uart_send("ERR:UNSUPPORTED_RATE");
        return;
    }

    s_current_sample_rate = sample_rate;
    status_uart_send_state();

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
        s_settle_until_us = esp_timer_get_time() + (AUDIO_SETTLING_DELAY_MS * 1000LL);
        ESP_LOGI(TAG,
                 "I2S reconfigured for %" PRIu32 " Hz; MUTE remains high for %d ms settling",
                 sample_rate,
                 AUDIO_SETTLING_DELAY_MS);
    }
    xSemaphoreGive(s_i2s_lock);
}

static void audio_apply_mute(uint8_t *buf, size_t len)
{
    if (s_usb_muted || s_volume == 0 || s_hw_muted) {
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

static int32_t audio_read_usb_sample_left_aligned_32(const uint8_t **buf,
                                                     uint32_t bits_per_sample,
                                                     uint32_t bytes_per_sample)
{
    if (bits_per_sample == 16 && bytes_per_sample == 2) {
        uint16_t raw = (uint16_t)(*buf)[0] | ((uint16_t)(*buf)[1] << 8);
        *buf += 2;
        return (int32_t)((uint32_t)raw << 16);
    }

    if (bits_per_sample == 24 && bytes_per_sample == 3) {
        uint32_t raw = (uint32_t)(*buf)[0] |
                       ((uint32_t)(*buf)[1] << 8) |
                       ((uint32_t)(*buf)[2] << 16);
        *buf += 3;

        if (raw & 0x00800000) {
            raw |= 0xff000000;
        }
        return (int32_t)(raw << 8);
    }

    if (bits_per_sample == 24 && bytes_per_sample == 4) {
        uint32_t raw = (uint32_t)(*buf)[0] |
                       ((uint32_t)(*buf)[1] << 8) |
                       ((uint32_t)(*buf)[2] << 16) |
                       ((uint32_t)(*buf)[3] << 24);
        *buf += 4;
        return (int32_t)raw;
    }

    *buf += bytes_per_sample;
    return 0;
}

static void audio_usb_to_i2s_32(const uint8_t *usb,
                                uint8_t *i2s,
                                size_t frames,
                                uint32_t bits_per_sample,
                                uint32_t bytes_per_sample)
{
    for (size_t frame = 0; frame < frames; frame++) {
        for (int channel = 0; channel < AUDIO_CHANNELS; channel++) {
            int32_t sample = audio_read_usb_sample_left_aligned_32(
                &usb,
                bits_per_sample,
                bytes_per_sample);
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
        sample *= 65536;
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
            if (!audio_can_run() || esp_timer_get_time() < s_settle_until_us) {
                vTaskDelay(pdMS_TO_TICKS(5));
            } else if (xStreamBufferBytesAvailable(s_audio_stream) >= audio_prefill_bytes()) {
                xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
                if (s_i2s_tx != NULL && i2s_channel_enable(s_i2s_tx) == ESP_OK) {
                    s_i2s_running = true;
                    ESP_LOGI(TAG, "I2S started at %" PRIu32 " Hz after %d ms prefill",
                             s_current_sample_rate,
                             CONFIG_AUDIO_PREFILL_MS);
                    if (!s_usb_muted && s_volume > 0) {
                        audio_set_hw_mute(false, "valid stream running");
                    }
                }
                xSemaphoreGive(s_i2s_lock);
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
#endif
            continue;
        }

        if (!audio_can_run()) {
            audio_stop("USB/VBUS/stream no longer active");
            continue;
        }

#if CONFIG_AUDIO_TEST_TONE
        audio_fill_test_tone(i2s_chunk, sizeof(i2s_chunk));
#else
        xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
        uint32_t bits_per_sample = s_current_bits_per_sample;
        uint32_t bytes_per_sample = s_current_usb_bytes_per_sample;
        uint32_t usb_bytes_per_frame = s_current_usb_bytes_per_frame;
        xSemaphoreGive(s_i2s_lock);

        size_t usb_chunk_bytes = AUDIO_I2S_CHUNK_FRAMES * usb_bytes_per_frame;
        size_t got = xStreamBufferReceive(s_audio_stream, usb_chunk, usb_chunk_bytes, pdMS_TO_TICKS(2));
        if (got < usb_chunk_bytes) {
            memset(usb_chunk + got, 0, usb_chunk_bytes - got);
            s_underruns++;
        }
        audio_usb_to_i2s_32(
            usb_chunk,
            i2s_chunk,
            AUDIO_I2S_CHUNK_FRAMES,
            bits_per_sample,
            bytes_per_sample);
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
            audio_set_hw_mute(true, "stream idle");
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

    if (!audio_can_run() || esp_timer_get_time() < s_settle_until_us) {
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
    s_usb_muted = mute != 0;
    if (s_usb_muted) {
        audio_set_hw_mute(true, "USB host mute");
    } else if (s_i2s_running && audio_can_run()) {
        audio_set_hw_mute(false, "USB host unmute");
    }
    ESP_LOGI(TAG, "USB mute set to %" PRIu32, mute);
}

static void uac_device_set_volume_cb(uint32_t volume, void *cb_ctx)
{
    (void)cb_ctx;
    s_volume = volume;
    if (volume == 0) {
        audio_set_hw_mute(true, "USB volume is zero");
    } else if (!s_usb_muted && s_i2s_running && audio_can_run()) {
        audio_set_hw_mute(false, "USB volume restored");
    }
    ESP_LOGI(TAG, "USB volume set to %" PRIu32, volume);
}

static void uac_device_set_sample_rate_cb(uint32_t sample_rate, void *cb_ctx)
{
    (void)cb_ctx;
    ESP_LOGI(TAG, "USB host selected sample rate: %" PRIu32 " Hz", sample_rate);
    audio_i2s_reconfigure(sample_rate);
}

static void uac_device_usb_state_cb(bool connected, void *cb_ctx)
{
    (void)cb_ctx;
    s_usb_connected = connected;
    ESP_LOGI(TAG, "USB host state: %s", connected ? "connected" : "disconnected");
    if (!connected) {
        s_stream_active = false;
        audio_stop("USB disconnected/suspended");
    }
    status_uart_send_state();
}

static void uac_device_stream_state_cb(bool active, void *cb_ctx)
{
    (void)cb_ctx;
    s_stream_active = active;
    ESP_LOGI(TAG, "USB audio stream: %s", active ? "started" : "stopped");
    if (!active) {
        audio_stop("USB stream stopped");
    } else {
        audio_set_hw_mute(true, "USB stream starting");
    }
    status_uart_send_state();
}

static void uac_device_set_format_cb(uint8_t bit_resolution, uint8_t bytes_per_sample, void *cb_ctx)
{
    (void)cb_ctx;

    if (!((bit_resolution == 16 && bytes_per_sample == 2) ||
          (bit_resolution == 24 && bytes_per_sample == 4))) {
        ESP_LOGW(TAG,
                 "Ignoring unsupported USB audio format: %u-bit, %u bytes/sample",
                 bit_resolution,
                 bytes_per_sample);
        audio_set_hw_mute(true, "unsupported USB format");
        status_uart_send("ERR:UNSUPPORTED_FORMAT");
        return;
    }

    audio_set_hw_mute(true, "USB format change");
    xSemaphoreTake(s_i2s_lock, portMAX_DELAY);
    s_current_bits_per_sample = bit_resolution;
    s_current_usb_bytes_per_sample = bytes_per_sample;
    s_current_usb_bytes_per_frame = AUDIO_CHANNELS * bytes_per_sample;

    if (s_i2s_tx != NULL && s_i2s_running) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_channel_disable(s_i2s_tx));
        s_i2s_running = false;
    }
    xSemaphoreGive(s_i2s_lock);

    if (s_audio_stream != NULL) {
        xStreamBufferReset(s_audio_stream);
    }

    ESP_LOGI(TAG,
             "USB host selected audio format: %u-bit, %u bytes/sample, %u bytes/frame; waiting for prefill",
             bit_resolution,
             bytes_per_sample,
             AUDIO_CHANNELS * bytes_per_sample);
    status_uart_send_state();
}

static void vbus_monitor_task(void *arg)
{
    (void)arg;
    bool last = s_vbus_present;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        bool present = gpio_get_level(PIN_VBUS_SENSE) != 0;
        if (present == last) {
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        present = gpio_get_level(PIN_VBUS_SENSE) != 0;
        if (present == last) {
            continue;
        }

        last = present;
        s_vbus_present = present;
        ESP_LOGI(TAG,
                 "USB VBUS %s on GPIO%d",
                 present ? "detected" : "removed",
                 PIN_VBUS_SENSE);
        if (!present) {
            audio_stop("VBUS removed");
        }
        status_uart_send_state();
    }
}

void app_main(void)
{
    board_status_io_init();
    status_uart_send("BOOT:ESP32S3_USB_I2S");
    status_uart_send_state();

    ESP_LOGI(TAG,
             "Starting USB Audio speaker: 44.1/48/88.2/96 kHz, 16/24-bit stereo, Philips I2S, "
             "external MCLK input on GPIO%d, oscillator select on GPIO%d",
             AUDIO_PIN_MCLK_IN,
             AUDIO_PIN_OSC_SELECT);
    ESP_LOGI(TAG,
             "Audio frame sizes: USB=4 or 8 bytes/frame, I2S=%d bytes/frame; at 48 kHz expect usb=192000 or 384000 B/s i2s=%d B/s",
             AUDIO_I2S_BYTES_PER_FRAME,
             48000 * AUDIO_I2S_BYTES_PER_FRAME);
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
        s_settle_until_us = esp_timer_get_time() + (AUDIO_SETTLING_DELAY_MS * 1000LL);
    } else {
        ESP_LOGE(TAG,
                 "I2S disabled; USB can still enumerate for descriptor testing");
    }

    s_audio_stream = xStreamBufferCreate(AUDIO_BUFFER_BYTES, AUDIO_USB_MAX_BYTES_PER_FRAME);
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

    task_ok = xTaskCreatePinnedToCore(
        vbus_monitor_task,
        "vbus_monitor",
        3072,
        NULL,
        4,
        NULL,
        tskNO_AFFINITY);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "failed to create VBUS monitor task");
        abort();
    }

    uac_device_config_t uac_config = {
        .skip_tinyusb_init = false,
        .output_cb = uac_device_output_cb,
        .input_cb = NULL,
        .set_mute_cb = uac_device_set_mute_cb,
        .set_volume_cb = uac_device_set_volume_cb,
        .set_sample_rate_cb = uac_device_set_sample_rate_cb,
        .set_format_cb = uac_device_set_format_cb,
        .usb_state_cb = uac_device_usb_state_cb,
        .stream_state_cb = uac_device_stream_state_cb,
        .cb_ctx = NULL,
    };

    ESP_ERROR_CHECK(uac_device_init(&uac_config));
    ESP_LOGI(TAG, "USB UAC initialized; waiting for host");
}
