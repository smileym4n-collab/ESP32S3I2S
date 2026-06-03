# ESP32-S3 USB to I2S DAC

ESP-IDF firmware for an ESP32-S3-N16R8 custom USB Audio Class speaker
that streams stereo PCM from native USB to a PCM1798 DAC over standard
Philips I2S.

## Hardware

- ESP32-S3-N16R8
- Native USB device port connected to the host
- PCM1798 DAC
- External fixed 12.288 MHz oscillator feeding the DAC MCLK directly
- ESP32-S3 outputs only BCLK, LRCK/WS, and DATA

When using PlatformIO, fill in the board-specific pins in
[platformio.ini](platformio.ini):

- `AUDIO_PIN_BCLK`
- `AUDIO_PIN_WS`
- `AUDIO_PIN_DATA`

When using native ESP-IDF instead, the same pins are available in
`idf.py menuconfig`:

- `USB to I2S DAC -> I2S BCLK GPIO`
- `USB to I2S DAC -> I2S LRCK/WS GPIO`
- `USB to I2S DAC -> I2S DATA GPIO`

The defaults are `-1` on purpose so the firmware fails early until the
pinout is known.

## Audio Format

The firmware presents to the host as `Tom Watson Audio` and supports:

- USB Audio Class speaker device through Espressif's TinyUSB-based
  `usb_device_uac` component
- 48 kHz and 96 kHz stereo, switchable by the OS
- 16-bit PCM
- Philips I2S
- 32-bit I2S slots, giving a conventional 64fs BCLK for external audio DACs
- No ESP32-S3 MCLK output

The 12.288 MHz oscillator is:

- 256fs at 48 kHz
- 128fs at 96 kHz

44.1 kHz and 88.2 kHz are not supported because this board does not have
an 11.2896 MHz-family clock.

When macOS changes the selected sample rate, the firmware stops I2S,
clears the audio buffer, recreates the I2S channel at the new rate, and
waits for prefill before restarting playback.

## VS Code / PlatformIO Build

Open this folder in VS Code with the PlatformIO extension installed.

Before flashing, edit [platformio.ini](platformio.ini) and replace the
three `-1` pin values:

```ini
build_flags =
    -DAUDIO_PIN_BCLK=4
    -DAUDIO_PIN_WS=5
    -DAUDIO_PIN_DATA=6
```

Keep the `-D` prefix, but make the GPIO value itself positive. For GPIO 4
use `-DAUDIO_PIN_BCLK=4`, not `-DAUDIO_PIN_BCLK=-4`.

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

Serial logs report USB mount/unmount from the UAC component, plus the
configured sample rate, I2S start/stop, mute/volume requests, and
software buffer underrun/overrun counts from the app.

## 24-bit Note

The code keeps the I2S side ready for 24-bit samples, but the shipped
default is 16-bit stereo while we bring up the board. Move to 24-bit only
after confirming the USB descriptors presented to the host advertise
24-bit speaker PCM cleanly on your operating system.
