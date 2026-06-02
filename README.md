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

The default build is a reliable first target:

- USB Audio Class speaker device through Espressif's TinyUSB-based
  `usb_device_uac` component
- 48 kHz stereo
- 16-bit PCM
- Philips I2S
- 32-bit I2S slots, giving a conventional 64fs BCLK for external audio DACs
- No ESP32-S3 MCLK output

The 12.288 MHz oscillator is:

- 256fs at 48 kHz
- 128fs at 96 kHz

44.1 kHz and 88.2 kHz are not supported because this board does not have
an 11.2896 MHz-family clock.

## 96 kHz Build

Use the alternate defaults file once 48 kHz is stable:

```sh
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults.96k" reconfigure
idf.py build
```

Or set both of these options in `menuconfig`:

- `USB Device UAC Configuration -> USB Device UAC -> UAC sample rate = 96000`
- `USB to I2S DAC -> USB/I2S sample rate = 96 kHz`

The firmware has a compile-time check so the USB descriptor rate and I2S
rate cannot silently diverge.

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

Then use the PlatformIO sidebar:

- `esp32s3_48k -> Build`
- `esp32s3_48k -> Upload`
- `esp32s3_48k -> Monitor`

For the experimental 96 kHz build, select the `esp32s3_96k` environment.

PlatformIO is used here only as the VS Code build/flash front-end. The
firmware still uses ESP-IDF and TinyUSB, not Arduino.

The first PlatformIO build will download the managed Espressif
`usb_device_uac` component, so the first build may take a little longer.

## Native ESP-IDF Build

```sh
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Serial logs report USB mount/unmount, the configured sample rate, I2S
start/stop, mute/volume requests, and software buffer underrun/overrun
counts.

## 24-bit Note

The code keeps the I2S side ready for 24-bit samples, but the default
managed Espressif `usb_device_uac` component exposes rate and channel
configuration more directly than bit-depth configuration. The shipped
default is therefore 16-bit stereo. Move to 24-bit only after confirming
the USB descriptors presented to the host advertise 24-bit speaker PCM,
or by switching to custom TinyUSB UAC descriptors.
