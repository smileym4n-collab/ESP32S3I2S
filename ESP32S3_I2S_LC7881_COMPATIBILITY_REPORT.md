# ESP32-S3 I2S Output and LC7881 Compatibility Report

## Scope

This report is based on inspection of the ESP32-S3 audio firmware in:

`/Users/tomwatson/Documents/GitHub/ESP32S3I2S`

The inspected checkout was:

- Branch: `codex/add-16-24-bit-audio-support`
- Commit: `b03a43c2abd9`

The inspection was read-only. The firmware was not modified, reformatted, built, flashed, or tested on hardware.

## Executive summary

The normal DAC/output firmware generates:

- Standard Philips I2S
- Normal one-BCLK delay between an LRCK transition and the following sample MSB
- Stereo: two slots per LRCK frame
- 32-bit physical slots
- 32-bit configured I2S data width
- 16-bit or 24-bit source audio aligned into the most-significant part of each 32-bit word
- 64 BCLKs per LRCK frame
- BCLK = 64fs at every supported sample rate

At the two most important rates:

- 44.1 kHz: BCLK = 2.8224 MHz, MCLK = 22.5792 MHz
- 48 kHz: BCLK = 3.072 MHz, MCLK = 24.576 MHz

The existing output does not provide a 48fs BCLK. If the Sanyo LC7881 requires a clock of exactly 48fs, the existing BCLK cannot be connected directly to that input.

The external oscillator/MCLK can be used as the reference from which the new DAC board derives 48fs, but there is an important distinction:

- If 48fs is only an independent LC7881 conversion clock, it can be derived from MCLK.
- If 48fs is also intended to shift the existing serial DATA stream, a separate clock alone is insufficient because the ESP32 DATA is framed against a 64fs BCLK. Interface logic would need to receive the 64fs Philips I2S stream and retime or reformat it for the LC7881.

## 1. Active I2S protocol

The active I2S initialization is `audio_i2s_init()` in:

`main/main.c`, starting around line 444.

Relevant configuration:

```c
i2s_chan_config_t chan_cfg =
    I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_AUDIO_I2S_PORT, I2S_ROLE_MASTER);

i2s_std_slot_config_t slot_cfg =
    I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_STEREO);

slot_cfg.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT;
slot_cfg.ws_width = 32;
```

This establishes:

- ESP32-S3 is the I2S master.
- The protocol is standard Philips I2S.
- It is not left-justified/MSB format.
- It is not right-justified.
- It is not PCM-short.
- It is not TDM.
- There are two slots per frame.
- Each physical slot is 32 BCLK periods.
- LRCK has a 50% duty cycle: 32 BCLKs low and 32 BCLKs high.

For ESP32-S3, the Philips helper macro establishes the important fields:

```c
.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
.slot_mode = I2S_SLOT_MODE_STEREO,
.slot_mask = I2S_STD_SLOT_BOTH,
.ws_pol = false,
.bit_shift = true,
.left_align = true,
.big_endian = false,
.bit_order_lsb = false
```

`bit_shift = true` is the field that enables the normal Philips one-bit shift. Therefore, after LRCK changes level, one BCLK period passes before the MSB of the new sample.

## 2. LRCK polarity and data timing

With the active defaults:

- LRCK/WS LOW represents the left channel.
- LRCK/WS HIGH represents the right channel.
- DATA is transmitted MSB-first.
- DATA changes or is launched on the falling edge of BCLK.
- The receiving DAC should sample DATA on the rising edge of BCLK.
- The sample MSB appears one BCLK after the associated LRCK transition.

The GPIO inversion configuration in `audio_i2s_init()` is:

```c
.invert_flags = {
    .mclk_inv = false,
    .bclk_inv = CONFIG_AUDIO_I2S_BCLK_INVERT,
    .ws_inv = CONFIG_AUDIO_I2S_WS_INVERT,
},
```

The active defaults in `sdkconfig.defaults`, lines 44-45, leave both inversion options disabled:

```text
# CONFIG_AUDIO_I2S_BCLK_INVERT is not set
# CONFIG_AUDIO_I2S_WS_INVERT is not set
```

Therefore the active build uses ordinary, non-inverted Philips I2S timing.

## 3. Sample and slot widths

There are two widths that should not be confused.

### Source audio resolution

The USB audio interface advertises:

- 16-bit PCM in a 2-byte USB subslot
- 24-bit PCM in a 4-byte USB subslot

This is configured in:

`components/usb_device_uac/tusb_uac/tusb_config_uac.h`, lines 32-50.

Relevant definitions:

```c
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_N_BYTES_PER_SAMPLE_RX  2
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX          16

#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_N_BYTES_PER_SAMPLE_RX  4
#define CFG_TUD_AUDIO_FUNC_1_FORMAT_2_RESOLUTION_RX          24
```

### I2S wire format

The I2S peripheral remains configured for:

- 32-bit data width
- 32-bit physical slot width
- Two slots per frame

The resulting frame contains:

```text
32 BCLK left slot + 32 BCLK right slot = 64 BCLK per LRCK frame
```

Therefore:

```text
BCLK / LRCK = 64
BCLK = 64fs
```

### Sample alignment and padding

The conversion is performed by:

- `audio_read_usb_sample_left_aligned_32()`
- `audio_usb_to_i2s_32()`

These functions are in `main/main.c`, around lines 620-670.

For 16-bit input:

```c
return (int32_t)((uint32_t)raw << 16);
```

The 16-bit sample occupies bits 31:16 of the I2S word. Bits 15:0 are zero padding.

For the active 24-bit format:

```c
if (bits_per_sample == 24 && bytes_per_sample == 4) {
    uint32_t raw = ...
    return (int32_t)raw;
}
```

The USB format has 24 significant bits in a 32-bit subslot. Those significant bits are expected to be MSB-aligned, leaving eight padding bits at the least-significant end.

Changing between 16- and 24-bit source audio does not alter:

- I2S protocol
- Slot width
- Number of slots
- BCLK frequency
- LRCK framing
- MCLK frequency

## 4. Supported sample rates

The supported rates are listed in:

`components/usb_device_uac/usb_device_uac.c`, lines 22-27.

```c
const uint32_t sample_rates[] = {
    44100,
    48000,
    88200,
    96000,
};
```

The default boot rate is 48 kHz, selected by:

`sdkconfig.defaults`, lines 18 and 29.

```text
CONFIG_UAC_SAMPLE_RATE=48000
CONFIG_AUDIO_SAMPLE_RATE_48000=y
```

The USB host can subsequently select any of the four supported rates at runtime.

## 5. External oscillators and MCLK

The firmware defines:

```c
#define AUDIO_OSC_44K_FAMILY_HZ 22579200
#define AUDIO_OSC_48K_FAMILY_HZ 24576000
```

`audio_clock_config_for_rate()` in `main/main.c`, around lines 358-385, selects the clock family:

```c
case 44100:
case 88200:
    cfg.oscillator_hz = 22579200;
    cfg.osc_select_high = false;
    break;

case 48000:
case 96000:
    cfg.oscillator_hz = 24576000;
    cfg.osc_select_high = true;
    break;
```

With the active, non-inverted oscillator-select setting:

- GPIO7 LOW selects 22.5792 MHz for 44.1 and 88.2 kHz.
- GPIO7 HIGH selects 24.576 MHz for 48 and 96 kHz.

The optional `CONFIG_AUDIO_OSC_SELECT_INVERT` setting could reverse this electrical polarity, but it is disabled in the active defaults.

### MCLK routing

The board documentation says that the selected oscillator is distributed by the 5PB1102 clock buffer to:

- ESP32-S3 GPIO15
- The external DAC MCLK/SCK header

GPIO15 is configured as an external clock input, not an output:

```c
.mclk = AUDIO_PIN_MCLK_IN,
```

The clock configuration is:

```c
std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_EXTERNAL;
std_cfg.clk_cfg.ext_clk_freq_hz = clock_cfg.oscillator_hz;
std_cfg.clk_cfg.mclk_multiple =
    audio_mclk_multiple(sample_rate, clock_cfg.oscillator_hz);
```

Therefore:

- The DAC MCLK header receives the selected oscillator directly.
- The ESP32 does not synthesize the DAC-header MCLK.
- The oscillator is not divided or multiplied before reaching the DAC MCLK header.
- GPIO15 is the ESP32 input for that same external reference.

### Internal ESP32 clock division

Inside the ESP32 I2S clock tree, `audio_mclk_multiple()` deliberately chooses:

```c
return (i2s_mclk_multiple_t)(oscillator_hz / sample_rate / 2);
```

This divides the external source by two internally:

- 22.5792 MHz becomes an internal 11.2896 MHz clock.
- 24.576 MHz becomes an internal 12.288 MHz clock.

The I2S peripheral derives BCLK and LRCK from this internal clock. This internal division does not modify the oscillator signal routed directly to the external DAC MCLK header.

## 6. Clock-frequency table

| Sample rate / LRCK | BCLK | External DAC MCLK | Physical slot width | Slots per frame | BCLK/LRCK | MCLK/LRCK | Internal ESP I2S clock |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 44.1 kHz | 2.8224 MHz | 22.5792 MHz | 32 bits | 2 | 64fs | 512fs | 11.2896 MHz / 256fs |
| 48 kHz | 3.072 MHz | 24.576 MHz | 32 bits | 2 | 64fs | 512fs | 12.288 MHz / 256fs |
| 88.2 kHz | 5.6448 MHz | 22.5792 MHz | 32 bits | 2 | 64fs | 256fs | 11.2896 MHz / 128fs |
| 96 kHz | 6.144 MHz | 24.576 MHz | 32 bits | 2 | 64fs | 256fs | 12.288 MHz / 128fs |

MCLK is therefore:

- 512fs at 44.1 kHz
- 512fs at 48 kHz
- 256fs at 88.2 kHz
- 256fs at 96 kHz

## 7. Clock continuity

### External MCLK

The external oscillator/MCLK is independent of the enabled state of the ESP32 I2S transmitter. The firmware has no oscillator-enable control.

It is therefore expected to remain running while a particular oscillator is selected, including when I2S BCLK and LRCK are stopped.

During a rate-family change, GPIO7 changes the oscillator mux selection. For example:

- 44.1 kHz to 48 kHz changes from 22.5792 MHz to 24.576 MHz.
- 48 kHz to 96 kHz keeps the 24.576 MHz oscillator selected.

The source code cannot prove whether the physical clock mux produces a brief gap, runt pulse, or glitch while changing between oscillators.

### BCLK and LRCK

BCLK and LRCK run only while the ESP-IDF I2S channel is enabled.

Normal runtime behavior:

- Short DMA underruns are filled with zero samples, so clocks continue.
- After 1000 ms without USB audio, the I2S channel is disabled and BCLK/LRCK stop.
- USB stream stop disables I2S.
- USB disconnect or suspend disables I2S.
- VBUS removal disables I2S.
- A source-format change between 16 and 24 bits disables I2S, clears the buffer, and waits for prefill before restarting.
- A sample-rate change disables and deletes the old I2S channel, changes the oscillator selection, recreates the channel, waits 500 ms for settling, then waits for audio prefill before restarting.

USB mute and zero volume normally do not stop I2S. Instead, hardware MUTE is asserted and zero sample data is transmitted while the channel continues to run.

## 8. Pin mapping

The active checked-out firmware uses:

| Signal | GPIO | Direction and notes |
|---|---:|---|
| BCLK | GPIO4 | ESP32 output |
| LRCK/WS | GPIO5 | ESP32 output |
| DATA | GPIO6 | ESP32 output |
| Oscillator select | GPIO7 | ESP32 output; LOW=22.5792 MHz, HIGH=24.576 MHz |
| External MCLK/SCK at ESP32 | GPIO15 | ESP32 input from the selected oscillator |
| MUTE | GPIO35 | Active-high output in the N8 production profile |
| Rate F3 | GPIO36 | Status output |
| Rate F2 | GPIO37 | Status output |
| Rate F1 | GPIO38 | Status output |
| Rate F0 | GPIO39 | Status output |

The DAC-header MCLK is a board-routed output from the external clock buffer. It is not an ESP32 MCLK output pin.

### N8R8 test-build limitation

The default PlatformIO environment is:

```ini
default_envs = n8r8_audio_test
```

The current N8R8 module uses GPIO35-37 for octal PSRAM. Consequently, this profile disables physical GPIO35-39 status/MUTE handling.

In the N8R8 profile:

- I2S clock and data pins remain active.
- Oscillator selection remains active.
- Physical MUTE on GPIO35 is not driven.
- MUTE remains a logical firmware state only.

The `esp32s3_n8` profile enables the physical MUTE and rate-status outputs for the intended non-PSRAM module.

## 9. Build and runtime variations

The current `platformio.ini` contains two targets:

1. `n8r8_audio_test`
2. `esp32s3_n8`

Both targets use the same:

- Philips I2S protocol
- 32-bit slots
- Two slots per frame
- 64fs BCLK
- GPIO4 BCLK
- GPIO5 LRCK
- GPIO6 DATA
- GPIO7 oscillator select
- GPIO15 external clock input

Their relevant difference is physical status/MUTE GPIO enablement.

Kconfig can alter:

- Default boot sample rate
- I2S peripheral port
- GPIO assignments when they are not overridden by build flags
- BCLK inversion
- LRCK/WS inversion
- Oscillator-select polarity
- Test-tone operation

There is no active compile-time or runtime option for:

- Left-justified output
- Right-justified output
- PCM output
- A 16-bit physical I2S slot
- A 24-bit physical I2S slot
- 32fs BCLK
- 48fs BCLK

The slot configuration is hard-coded to 32-bit stereo Philips I2S.

The local `main` branch at commit `e61491b1c024` also uses 32-bit stereo Philips I2S and 64fs BCLK. It represents an older hardware profile with oscillator selection on GPIO17 and fixed 16-bit USB input. Its I2S framing and clock ratios are unchanged.

No Zeppelin or I2S-slave target exists in the checked-out files, local branches, fetched remote branches, or available repository history. The only I2S role configured by this repository is `I2S_ROLE_MASTER`.

## 10. LC7881 compatibility

The existing clock ratios are:

- BCLK = 64fs
- MCLK = 512fs at 44.1 and 48 kHz
- MCLK = 256fs at 88.2 and 96 kHz

The clock required by the LC7881 conversion board is stated to be exactly 48fs.

| Sample rate | Existing 64fs BCLK | Desired 48fs clock |
|---:|---:|---:|
| 44.1 kHz | 2.8224 MHz | 2.1168 MHz |
| 48 kHz | 3.072 MHz | 2.304 MHz |
| 88.2 kHz | 5.6448 MHz | 4.2336 MHz |
| 96 kHz | 6.144 MHz | 4.608 MHz |

### Direct BCLK connection

The existing BCLK cannot be connected directly to an LC7881 clock input that requires exactly 48fs:

```text
Existing BCLK = 64fs
Required clock = 48fs
```

### Deriving 48fs from MCLK

At 44.1 and 48 kHz:

```text
MCLK = 512fs
48fs = MCLK × 48/512
48fs = MCLK × 3/32
```

At 88.2 and 96 kHz:

```text
MCLK = 256fs
48fs = MCLK × 48/256
48fs = MCLK × 3/16
```

This is not achievable with a single ordinary integer divider. The DAC board would need a suitable fractional divider, PLL, programmable clock generator, counter arrangement implementing the rational ratio, or other clock-synthesis logic.

### Serial-data compatibility warning

The existing DATA line is serialized against the existing 64fs BCLK:

```text
32-bit left slot + 32-bit right slot = 64 BCLKs per frame
```

If the LC7881 expects its 48fs clock to shift or frame that DATA directly, generating a separate 48fs clock does not make the existing DATA stream compatible. The 48fs clock would not have enough cycles to preserve the existing 64-bit frame.

In that case, the drop-in board needs interface logic that:

1. Receives standard 64fs Philips I2S from the ESP32.
2. Extracts the left and right audio samples.
3. Converts the samples into the framing required by the LC7881.
4. Generates the exact 48fs LC7881 clock.

If the LC7881 48fs input is instead an independent conversion clock and does not shift the serial I2S DATA, then it can be derived separately from MCLK using the ratios above.

## Final conclusion

The normal ESP32-S3 DAC/output build produces standard stereo Philips I2S with 32-bit physical slots and a fixed 64fs BCLK.

It provides:

- 64fs BCLK directly
- Not 48fs
- Not 32fs

The existing BCLK is therefore not a direct match for an LC7881 input requiring exactly 48fs.

The external MCLK is a suitable synchronous reference for generating 48fs, but whether clock generation alone is sufficient depends on the role of the LC7881 48fs input. If it is the serial-data shift clock, the board must also receive and reformat the existing 64fs I2S data stream.
