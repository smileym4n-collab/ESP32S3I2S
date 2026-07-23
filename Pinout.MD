# ESP32-S3 USB Audio I²S Pinout

This board is a self-powered ESP32-S3-WROOM-1U USB Audio Class to I²S
bridge. The current assembly has an N8R8 module; the intended replacement is
an N8 module without octal PSRAM.

## GPIO Map

| Function | GPIO | Direction | Notes |
| --- | ---: | --- | --- |
| I²S BCLK | 4 | Output | Bit clock to DAC |
| I²S LRCK / WS | 5 | Output | Word-select clock to DAC |
| I²S DATA | 6 | Output | Serial audio data to DAC |
| FREQ / oscillator select | 7 | Output | LOW = 22.5792 MHz, HIGH = 24.5760 MHz |
| I²C SDA | 8 | Reserved | Defined for future board control; not initialized |
| I²C SCL | 9 | Reserved | Defined for future board control; not initialized |
| ATTINY_UART_TX | 10 | Output | UART1 TX to ATtiny1616 RX, 9600 baud 8N1 |
| VBUS_SENSE | 13 | Input | Active-high host VBUS indication through external divider |
| MCLKESP / external SCK-MCLK | 15 | Input | Selected clock from 5PB1102 |
| Native USB D- | 19 | USB | Fixed ESP32-S3 native USB pin |
| Native USB D+ | 20 | USB | Fixed ESP32-S3 native USB pin |
| MUTE | 35 | Output | Active-high mute/status |
| F3 | 36 | Output | Sample-rate status MSB |
| F2 | 37 | Output | Sample-rate status |
| F1 | 38 | Output | Sample-rate status |
| F0 | 39 | Output | Sample-rate status LSB |

UART0 RX/TX are intentionally not assigned to ATtiny status and remain
available for the normal programming and debug console.

## N8R8 Audio-Test Profile

Espressif specifies GPIO35, GPIO36, and GPIO37 as internally connected to
octal PSRAM and unavailable for other uses on ESP32-S3R8 WROOM-1/1U modules.
That includes ESP32-S3-WROOM-1U-N8R8 and conflicts with this PCB's requested
MUTE, F3, and F2 assignments.

The default `n8r8_audio_test` PlatformIO profile disables PSRAM initialization
and does not configure any of GPIO35-GPIO39. It is intended only to validate
USB enumeration, sample-rate switching, external MCLK, BCLK/LRCK/DATA,
oscillator selection, VBUS sensing, and the ATtiny UART. MUTE and F0-F3 are
logical/logged states only in this profile and do not drive the PCB.

The `esp32s3_n8` profile enables GPIO35-GPIO39 for the future non-PSRAM N8
module. No application-code rewrite is required when the module changes;
selecting the N8 build environment enables the full status interface.

## Clock Selection and Supported Rates

The selected 22.5792 MHz or 24.5760 MHz oscillator is distributed by the
5PB1102 to both the DAC SCK/MCLK header and ESP32-S3 GPIO15.

| Sample Rate | GPIO7 | Oscillator | MCLK Ratio | F3 | F2 | F1 | F0 |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 44.1 kHz | LOW | 22.5792 MHz | 512 fs | 0 | 0 | 0 | 1 |
| 48 kHz | HIGH | 24.5760 MHz | 512 fs | 0 | 0 | 1 | 0 |
| 88.2 kHz | LOW | 22.5792 MHz | 256 fs | 0 | 0 | 1 | 1 |
| 96 kHz | HIGH | 24.5760 MHz | 256 fs | 0 | 1 | 0 | 0 |

176.4 kHz and 192 kHz are not advertised or supported. During inactive,
disconnected, muted/reconfiguring, or unsupported states, F3..F0 are all
LOW.

Firmware asserts MUTE before selecting GPIO7 and reconfiguring I²S. It waits
500 ms for the clock path to settle, waits for valid USB audio prefill, then
drives MUTE LOW only after I²S is running. MUTE is HIGH on USB/VBUS
disconnect, stream stop, unsupported format/rate, and I²S errors.

## External MCLK

The detected ESP-IDF standard-I²S driver genuinely supports external clock
input on ESP32-S3. Firmware configures:

```c
clk_src = I2S_CLK_SRC_EXTERNAL;
ext_clk_freq_hz = 22579200; /* or 24576000 */
gpio_cfg.mclk = GPIO_NUM_15;
```

With that source selected, the driver treats the MCLK pin as an input and
derives BCLK and LRCK/WS from the external clock. There is no internal-clock
fallback. If the selected target/ESP-IDF build does not expose this path, or
I²S initialization fails, firmware logs the limitation, keeps MUTE HIGH,
leaves F3..F0 LOW, and does not start I²S.

## VBUS Sense

GPIO13 reads USB VBUS through the PCB resistor divider; it is not 5 V
tolerant and must not be connected directly to VBUS. Firmware checks it at
boot and polls it during runtime. Without VBUS, audio stays stopped and
muted. When VBUS appears, USB mount plus a valid stream can enable audio.
When it disappears, audio is stopped immediately and status returns to the
safe inactive state. VBUS is treated only as host-presence detection, not as
the board power source.

## ATtiny1616 Status UART

GPIO10 uses UART1 TX at 9600 baud, 8 data bits, no parity, and 1 stop bit.
Each message is ASCII terminated by CRLF. Examples:

```text
BOOT:ESP32S3_USB_I2S
SR:44100,BITS:16,MUTE:0,USB:1
SR:48000,BITS:24,MUTE:0,USB:1
USB:0,MUTE:1
ERR:UNSUPPORTED_RATE
ERR:UNSUPPORTED_FORMAT
```

State messages are sent on boot and when USB, sample format/rate, or MUTE
state changes.
