# ESP32-S3 USB to I2S DAC

ESP-IDF firmware for an ESP32-S3-N16R8 custom USB Audio Class speaker
that streams stereo PCM from native USB to an external DAC over standard
Philips I2S.

## Hardware

- ESP32-S3-N16R8
- Native USB device port connected to the host
- External I2S DAC
- 22.5792 MHz oscillator for the 44.1/88.2 kHz family
- 24.5760 MHz oscillator for the 48/96 kHz family
- 74LVC1G157GW oscillator mux selected by ESP32-S3 D17
- 5PB1102PGGK clock buffer feeding SCK/MCLK to ESP32-S3 D15 and the DAC header
- ESP32-S3 outputs BCLK, LRCK/WS, and DATA

Board pins:

- D4: I2S BCLK output
- D5: I2S LRCK/WS output
- D6: I2S DATA output
- D15: external SCK/MCLK input from the 5PB1102PGGK
- D17: oscillator select output

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
- 16-bit PCM
- Philips I2S
- 32-bit I2S slots, giving a conventional 64fs BCLK for external audio DACs
- External MCLK input on D15 using ESP-IDF `I2S_CLK_SRC_EXTERNAL`

Oscillator selection:

- 44.1 kHz: D17 LOW, 22.5792 MHz, 512fs
- 48 kHz: D17 HIGH, 24.5760 MHz, 512fs
- 88.2 kHz: D17 LOW, 22.5792 MHz, 256fs
- 96 kHz: D17 HIGH, 24.5760 MHz, 256fs

176.4 kHz and 192 kHz are not advertised or handled in this initial firmware.

When macOS changes the selected sample rate, the firmware stops I2S,
clears the audio buffer, selects the correct oscillator on D17, recreates
the I2S channel at the new rate, and waits for prefill before restarting
playback.

## External MCLK Requirement

This PCB expects the ESP32-S3 I2S peripheral to derive BCLK and LRCK/WS
from the selected oscillator arriving on D15. The firmware does not fall
back to internally generated I2S clocks if external MCLK input support is
not available, because that could leave the ESP32-S3 and DAC clocked from
different sources.

ESP-IDF documents external I2S clock input through `I2S_CLK_SRC_EXTERNAL`
and `ext_clk_freq_hz`, with the MCLK pin becoming an input when that clock
source is selected. If a target or ESP-IDF version does not expose that
path, the firmware logs a clear error and leaves I2S playback disabled.

## VS Code / PlatformIO Build

Open this folder in VS Code with the PlatformIO extension installed.

Then use the PlatformIO sidebar:

- `esp32s3_audio -> Build`
- `esp32s3_audio -> Upload`
- `esp32s3_audio -> Monitor`

PlatformIO is used here only as the VS Code build/flash front-end. The
firmware still uses ESP-IDF and TinyUSB, not Arduino.

The Espressif `usb_device_uac` component is vendored in this repo with a
small CMake adjustment for PlatformIO. The first build may still take a
little longer while PlatformIO prepares ESP-IDF.

## Native ESP-IDF Build

```sh
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Serial logs report USB mount/unmount from the UAC component, selected
sample rate, oscillator family, D17 level, I2S pins, whether external
MCLK input is in use, I2S start/stop, mute/volume requests, and software
buffer underrun/overrun counts from the app.

## 24-bit Note

The code keeps the I2S side ready for 24-bit samples, but the shipped
default is 16-bit stereo while we bring up the board. Move to 24-bit only
after confirming the USB descriptors presented to the host advertise
24-bit speaker PCM cleanly on your operating system.
