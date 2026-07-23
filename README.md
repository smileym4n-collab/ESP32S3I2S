# ESP32-S3 USB to I2S DAC

ESP-IDF firmware for an ESP32-S3-WROOM-1U custom USB Audio Class speaker
that streams stereo PCM from native USB to an external DAC over standard
Philips I2S. It includes separate profiles for the current N8R8 test module
and the future N8 production module.

## Hardware

- Current: ESP32-S3-WROOM-1U-N8R8, USB/I²S audio testing only
- Future: ESP32-S3-WROOM-1U-N8, with full MUTE and F0-F3 outputs
- Native USB device port connected to the host
- External I2S DAC
- 22.5792 MHz oscillator for the 44.1/88.2 kHz family
- 24.5760 MHz oscillator for the 48/96 kHz family
- Oscillator mux selected by ESP32-S3 GPIO7
- 5PB1102PGGK clock buffer feeding SCK/MCLK to ESP32-S3 GPIO15 and the DAC header
- ESP32-S3 outputs BCLK, LRCK/WS, and DATA

Board pins:

- GPIO4: I2S BCLK output
- GPIO5: I2S LRCK/WS output
- GPIO6: I2S DATA output
- GPIO7: oscillator select output
- GPIO8/GPIO9: reserved I2C SDA/SCL
- GPIO10: 9600-baud UART1 TX to ATtiny1616 RX
- GPIO13: VBUS sense input through the PCB resistor divider
- GPIO15: external SCK/MCLK input from the 5PB1102PGGK
- GPIO19/GPIO20: native USB D-/D+
- GPIO35: active-high MUTE/status
- GPIO36/GPIO37/GPIO38/GPIO39: sample-rate status F3/F2/F1/F0

UART0 RX/TX are not used for ATtiny status and remain available for the
normal programming/debug console.

> **N8R8 test limitation:** GPIO35, GPIO36, and GPIO37 are connected to octal
> PSRAM on the current module. The default `n8r8_audio_test` profile disables
> PSRAM initialization and compiles out every GPIO35-GPIO39 access. USB, I²S,
> oscillator selection, VBUS sense, and ATtiny UART remain available, but
> physical MUTE and F0-F3 do not operate. The future `esp32s3_n8` profile
> enables all five outputs without source-code changes.

When using native ESP-IDF, the pins are available in
`idf.py menuconfig`:

- `USB to I2S DAC -> I2S BCLK GPIO`
- `USB to I2S DAC -> I2S LRCK/WS GPIO`
- `USB to I2S DAC -> I2S DATA GPIO`
- `USB to I2S DAC -> I2S external SCK/MCLK input GPIO`
- `USB to I2S DAC -> Oscillator select GPIO`

## Audio Format

The firmware presents to the host as `Tom Watson Audio` and supports:

- USB Audio Class speaker device through Espressif's TinyUSB-based
  `usb_device_uac` component
- 44.1 kHz, 48 kHz, 88.2 kHz, and 96 kHz stereo, switchable by the OS
- 16-bit and 24-bit PCM, selectable by the OS from the same firmware
- Philips I2S
- 32-bit I2S slots, giving a conventional 64fs BCLK for external audio DACs
- External MCLK input on GPIO15 using ESP-IDF `I2S_CLK_SRC_EXTERNAL`

Oscillator selection:

- 44.1 kHz: GPIO7 LOW, 22.5792 MHz, 512fs
- 48 kHz: GPIO7 HIGH, 24.5760 MHz, 512fs
- 88.2 kHz: GPIO7 LOW, 22.5792 MHz, 256fs
- 96 kHz: GPIO7 HIGH, 24.5760 MHz, 256fs

176.4 kHz and 192 kHz are not advertised or handled in this initial firmware.

When macOS changes the selected sample rate, the firmware stops I2S,
clears the audio buffer, selects the correct oscillator on GPIO7, recreates
the I2S channel at the new rate, and waits for prefill before restarting
playback. When the OS changes between 16-bit and 24-bit formats, the
firmware clears the audio buffer and restarts I2S after the normal prefill.

In the N8 production profile, GPIO35 remains HIGH during disconnect, stream stop, unsupported state, and
clock/I2S reconfiguration. After a rate change, firmware waits 500 ms for
the clock path to settle, starts valid I2S playback after prefill, then
drives MUTE LOW. F3..F0 are all LOW while muted/inactive and show `0001`,
`0010`, `0011`, or `0100` for 44.1, 48, 88.2, or 96 kHz respectively.

## External MCLK Requirement

This PCB expects the ESP32-S3 I2S peripheral to derive BCLK and LRCK/WS
from the selected oscillator arriving on GPIO15. The firmware does not fall
back to internally generated I2S clocks if external MCLK input support is
not available, because that could leave the ESP32-S3 and DAC clocked from
different sources.

ESP-IDF documents external I2S clock input through `I2S_CLK_SRC_EXTERNAL`
and `ext_clk_freq_hz`, with the MCLK pin becoming an input when that clock
source is selected. If a target or ESP-IDF version does not expose that
path, the firmware logs a clear error and leaves I2S playback disabled.

The ESP-IDF version used by the detected PlatformIO environment does expose
this standard-I2S external-clock path for ESP32-S3. The firmware sets
`clk_src = I2S_CLK_SRC_EXTERNAL`, supplies `ext_clk_freq_hz`, and configures
GPIO15 as the I2S MCLK input. This is not an internally generated fallback.

## VBUS and ATtiny Status

GPIO13 is sampled at boot and polled during runtime. A LOW level means host
VBUS is absent: audio is stopped, MUTE is HIGH, and F3..F0 are LOW. A HIGH
level means VBUS was detected through the external divider and allows a
mounted USB stream to run. The firmware does not assume VBUS powers this
self-powered board.

UART1 sends CRLF-terminated, 9600-baud 8N1 lines on GPIO10. Messages include:

```text
BOOT:ESP32S3_USB_I2S
SR:44100,BITS:16,MUTE:0,USB:1
SR:48000,BITS:24,MUTE:0,USB:1
USB:0,MUTE:1
ERR:UNSUPPORTED_RATE
ERR:UNSUPPORTED_FORMAT
```

State lines are sent on USB, rate/format, and MUTE changes. `BITS` reflects
the existing host-selected 16-bit or 24-bit USB format.

## VS Code / PlatformIO Build

Open this folder in VS Code with the PlatformIO extension installed.

The PlatformIO sidebar exposes:

- `n8r8_audio_test`: current module, safe core-audio testing
- `esp32s3_n8`: future N8 module, full MUTE/F output support

PlatformIO is used here only as the VS Code build/flash front-end. The
firmware still uses ESP-IDF and TinyUSB, not Arduino.

For the current N8R8 board, this is the default:

```sh
pio run
pio run -t upload
pio device monitor
```

The equivalent explicit commands are:

```sh
pio run -e n8r8_audio_test
pio run -e n8r8_audio_test -t upload
pio device monitor -e n8r8_audio_test
```

After installing the N8 module:

```sh
pio run -e esp32s3_n8
pio run -e esp32s3_n8 -t upload
pio device monitor -e esp32s3_n8
```

On macOS, find likely ports with `ls /dev/cu.*`; on Linux use
`ls /dev/ttyUSB* /dev/ttyACM*`.

The Espressif `usb_device_uac` component is vendored in this repo with a
small CMake adjustment for PlatformIO. The first build may still take a
little longer while PlatformIO prepares ESP-IDF.

## Native ESP-IDF Build

```sh
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash
idf.py -p <PORT> monitor
```

Serial logs report USB mount/unmount from the UAC component, selected
sample rate, selected 16/24-bit audio format, oscillator family, GPIO7 level,
I2S pins, whether external MCLK input is in use, I2S start/stop, mute/volume
requests, and software buffer underrun/overrun counts from the app.

## 24-bit Note

The USB speaker interface advertises two streaming formats: 16-bit stereo
PCM and 24-bit stereo PCM in 32-bit slots. The I2S side remains 32-bit
stereo Philips I2S for both modes, so BCLK stays at the conventional 64fs
rate while the audio payload changes with the OS-selected format.
