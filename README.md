# Air Conditioner Controller (ESP32-C3 + IR + MQTT)

Learn the ON and OFF infrared commands of any air-conditioner remote, then replay them remotely by sending a JSON message to an MQTT broker (EMQX Cloud).

- **Learn mode** on the BOOT button: point the original remote at the receiver and the device records the raw IR signal.
- **Protocol agnostic**: the signals are stored as raw timings (microseconds) and replayed at 38 kHz, so it works with any remote (LG, Daikin, Mitsubishi, ...) without decoding it.
- **Remote control** over MQTT/TLS with `{ "command": "on" }` / `{ "command": "off" }`.
- **Persistent**: payloads are stored in flash (NVS) and survive reboots.
- **Feedback** on the 0.42" OLED, on the board LED and on the Serial Monitor.

## Hardware

| Part | Notes |
|---|---|
| ESP32-C3 board with built-in 0.42" OLED (72x40, SSD1306) | OLED on I2C: SDA = GPIO5, SCL = GPIO6 |
| IR receiver board | Module with demodulator (VS1838B / TSOP type, active-low output) |
| IR LED 940 nm | About 20 mA nominal, up to 30-35 mA |
| NPN transistor | 2N2222 / BC337 or similar |
| Resistors | 1 kΩ (base), 120 Ω (LED) |
| Optional | 47 µF capacitor between 5V and GND near the LED |

### Pin map

| GPIO | Function |
|---|---|
| 3 | IR transmitter output (to transistor base through 1 kΩ) |
| 4 | IR receiver power enable (HIGH only while learning) |
| 5 / 6 | OLED SDA / SCL (on-board) |
| 8 | On-board LED (assumed active-low, see `LED_ACTIVE_LOW`) |
| 9 | BOOT button (on-board) |
| 10 | IR receiver output |

### IR LED wiring

A GPIO cannot supply 30 mA safely, so the LED is switched by a transistor:

```
5V ──[ 120 Ω ]── LED anode
                 LED cathode ── NPN collector
GPIO3 ──[ 1 kΩ ]── NPN base
                   NPN emitter ── GND
```

- With a 5 V supply, a 1.4 V LED and about 0.2 V across the transistor, 120 Ω gives about 28 mA. Use 150 Ω for about 22 mA. Do not go below 100 Ω.
- Powering the LED from 3.3 V also works with about 56 Ω.
- A 1 kΩ base resistor gives about 2.6 mA of base current, plenty to saturate the transistor.

### IR receiver power control

The receiver board is powered only in learn mode, so it cannot pick up stray signals (or the device's own LED) while transmitting. GPIO4 (`PIN_IR_RX_EN`) is HIGH only while learning.

**Option A (recommended): power the board directly from GPIO4.** A typical receiver module draws 1-5 mA, well within what a GPIO can supply:

```
GPIO4 ── receiver VCC
GND   ── receiver GND
GPIO10 ── receiver OUT
```

**Option B: low-side NPN switch**, if the board draws more than about 10 mA:

```
3V3 ── receiver VCC
receiver GND ── NPN collector
NPN emitter ── GND
GPIO4 ──[ 1 kΩ ]── NPN base
GPIO10 ── receiver OUT
```

With option B the board's ground floats while it is off, so the OUT line is undefined. This is harmless because the firmware ignores the receiver in transmit mode.

## Software requirements

Arduino IDE (or Arduino CLI) with the ESP32 board package, and these libraries:

- `IRremoteESP8266`
- `PubSubClient`
- `ArduinoJson` (version 7, uses `JsonDocument`)
- `U8g2`

Board settings in the Arduino IDE:

- Board: an ESP32-C3 board (for example *ESP32C3 Dev Module*)
- **USB CDC On Boot: Enabled** (required for `Serial` output over USB)
- Serial Monitor: **115200 baud**

## Configuration

Edit the user section at the top of `Air_Conditioner_Controller.ino`:

```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const char* MQTT_HOST     = "your-broker.emqxsl.com";
const uint16_t MQTT_PORT  = 8883;
const char* MQTT_USERNAME = "USERNAME";
const char* MQTT_PASSWORD = "PASSWORD";
```

The built-in CA certificate is **DigiCert Global Root G2**, which is the root used by EMQX Cloud. If your broker uses a different CA, replace `ROOT_CA`.

Do not commit real credentials to a public repository.

## Usage

### Learning the remote's commands

1. Press **BOOT**. The LED starts blinking and the display shows `Comando:` / `ON`.
2. Point the original remote at the receiver and press its **ON** button.
3. The display switches to `Comando:` / `OFF`. Press the remote's **OFF** button.
4. The device shows `Salvato OK` and returns to transmit mode (LED solid).

Pressing **BOOT** at any moment during learning cancels it and returns to transmit mode. Anything already saved is kept.

Learning protections:

- Signals shorter than 16 timings are discarded as noise.
- Repeated frames of the same key are ignored for 1.5 s after a capture.
- If the OFF signal is identical to the ON one, it is ignored and the device keeps waiting.

### Controlling over MQTT

The topics are derived from the chip ID (printed on the Serial Monitor at boot):

| Topic | Direction | Purpose |
|---|---|---|
| `devices/<deviceId>/cmd` | to device | commands |
| `devices/<deviceId>/status` | from device | replies |

Commands:

```json
{ "command": "on" }
{ "command": "off" }
```

Replies:

| Situation | Reply |
|---|---|
| Command sent | `{"status":"ok"}` |
| Unknown command | `{"status":"failed","error":"Command unknown"}` |
| Payload never learned | `{"status":"failed","error":"Payload not configured"}` |
| Device in learn mode | `{"status":"failed","error":"Learning mode active"}` |
| Malformed JSON | `{"status":"failed","error":"Invalid JSON"}` |

Commands are case-insensitive (`"ON"` works too).

### Feedback

| Where | What |
|---|---|
| Display, transmit mode | `WiFi OK` / `No WiFi` on the first line, `MQTT` / `...` on the second |
| Display, learn mode | `Comando:` and `ON` or `OFF` |
| Display, command received | `RECEIVED` and the command, for 1.5 s |
| LED solid | Transmit mode |
| LED blinking | Learn mode |
| LED two quick flashes | MQTT command received |

### Serial Monitor log

Every line carries a local timestamp (from NTP; uptime is shown until the clock syncs):

```
[2026-11-19 14:03:00] Learn Mode: ON (waiting for remote's ON button)
[2026-11-19 14:03:04] IR signal received: LG, 28 bits, 59 raw timing
[2026-11-19 14:03:04] Received Payload: ON  -> (59 timing): 3150,9850,550,1550,...
[2026-11-19 14:03:09] Received Payload: OFF -> (59 timing): 3150,9850,550,550,...
[2026-11-19 14:03:10] Learn Mode: OFF -> transmit mode
[2026-11-19 14:05:22] Received Command (MQTT): 2026-11-19 14:05:22 {"command":"on"}  [topic devices/.../cmd]
[2026-11-19 14:05:22] IR sent: on (59 timing @ 38 kHz)
[2026-11-19 14:05:22] MQTT response sent -> {"status":"ok"}
```

## How it works

- **Capture**: `IRrecv` with a 1024-entry buffer and a 50 ms timeout (long enough for the big state messages of air conditioners). The raw timings are copied into a `std::vector<uint16_t>`.
- **Storage**: the vectors are saved as binary blobs in NVS (`Preferences`, namespace `ir`, keys `on` and `off`).
- **Replay**: `IRsend::sendRaw(timings, length, 38)`, with the carrier generated in software on GPIO3.
- **Connectivity**: WiFi, NTP and MQTT are managed without blocking, so the BOOT button and learn mode work even when the network is down. No MQTT (TLS) connection is attempted while learning, because the handshake blocks for a moment.
- **Time**: NTP (timezone `CET-1CEST`) is needed for the TLS certificate validation and for the log timestamps.

## Troubleshooting

| Problem | Likely cause / fix |
|---|---|
| No output on the Serial Monitor | Enable *USB CDC On Boot*, set 115200 baud, press RST |
| `GPIO isr service is not installed` | Caused by calling `disableIRIn()` before `enableIRIn()`. Already fixed in this version |
| Nothing is captured | Check the receiver wiring on GPIO10 and that it is a demodulating module. Try the `IRrecvDumpV3` example |
| ON and OFF seem to be learned wrongly | Hold the remote 10-20 cm from the receiver and press the key once |
| AC does not react to the replay | Check the LED wiring and resistor, point the LED at the AC, try the 5 V supply |
| Board LED is inverted | Set `LED_ACTIVE_LOW = false` |
| MQTT keeps failing | Check host, credentials, and that NTP has synced (the log says `syncing NTP...`) |
| Compile errors from `IRremoteESP8266` | Library support for ESP32-C3 depends on the library and ESP32 core versions. Update both |

## Project layout

```
Air_Conditioner_Controller/
├── Air_Conditioner_Controller.ino   # whole firmware
└── README.md
```

## Limitations

- Only two commands (`on`, `off`) are supported. Adding more means adding more keys, for example `cool_24`, with the same learn flow.
- Raw replay is bit-banged in software, so timing can jitter slightly under heavy WiFi activity. It is fine for typical AC remotes.
- Remotes that send different data on every press (rolling codes) cannot be replayed. AC remotes normally do not.
