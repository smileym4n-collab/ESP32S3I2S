/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "uac_descriptors.h"

//------------- CLASS -------------//
// The number of audio interfaces
#define CFG_TUD_AUDIO             1

//--------------------------------------------------------------------
// AUDIO CLASS DRIVER CONFIGURATION
//--------------------------------------------------------------------

// Enable feedback EP
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP                             1

// Enable/disable conversion from 16.16 to 10.14 format on full-speed devices
#if CONFIG_UAC_SUPPORT_MACOS
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_FORMAT_CORRECTION              1
#endif

#define CFG_TUD_AUDIO_FUNC_1_DESC_LEN                                TUD_AUDIO_DEVICE_DESC_LEN

// Keep one active streaming alternate setting for reliable ESP32-S3/macOS
// operation. USB carries packed 24-bit PCM (3-byte subslots) and the I2S side
// expands those samples into 32-bit Philips-I2S slots.
#define CFG_TUD_AUDIO_FUNC_1_N_FORMATS                               1

// ESP32-S3 full-speed USB has a practical 512-byte endpoint packet limit.
// Packed 24-bit stereo at 48 kHz needs at most 294 bytes per 1 ms packet.
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE                         48000
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX                           MIC_CHANNEL_NUM
#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX                           SPEAK_CHANNEL_NUM

// TX is unused by this speaker-only firmware but remains Kconfig-driven for
// the generic component path.
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX          CONFIG_UAC_BYTES_PER_SAMPLE
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_TX                  CONFIG_UAC_BIT_RESOLUTION
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX          3
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX                  24
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_TX                    (CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_TX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX)
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_RX                    (CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX * CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX)

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_IN                1

// MIC IN EP (device-to-host = TX): keep EP size aligned to one complete audio frame.
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN    ((CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE / 1000 * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_TX) + CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_TX)

#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ      CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN * (MIC_INTERVAL_MS + 1)
#define CFG_TUD_AUDIO_FUNC_1_EP_IN_SZ_MAX         CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_IN  // Maximum EP IN size for all AS alternate settings used

// EP and buffer size - for isochronous EP´s, the buffer and EP size are equal (different sizes would not make sense)
#define CFG_TUD_AUDIO_ENABLE_EP_OUT               1

// SPK OUT EP (host-to-device = RX): keep EP size aligned to one complete audio frame.
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT   ((CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE / 1000 * CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_RX) + CFG_TUD_AUDIO_FUNC_1_FORMAT_1_FRAME_SZ_RX)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ     CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT * (SPK_INTERVAL_MS + 1)
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX        CFG_TUD_AUDIO_FUNC_1_FORMAT_1_EP_SZ_OUT // Maximum EP OUT size for all AS alternate settings used

// Number of Standard AS Interface Descriptors (4.9.1) defined per audio function - this is required to be able to remember the current alternate settings of these interfaces - We restrict us here to have a constant number for all audio functions (which means this has to be the maximum number of AS interfaces an audio function has and a second audio function with less AS interfaces just wastes a few bytes)
#define CFG_TUD_AUDIO_FUNC_1_N_AS_INT             1

// Size of control request buffer
#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ    64

#ifdef __cplusplus
}
#endif
