# Embedded Computing 2 Lab

# Task 4: UART Bridge + XPLORER Board

Task 4 extends the MQTT communication chain from Task 3 by adding a physical UART bridge between an Arduino Nano 33 IoT and a Raspberry Pi Pico mounted on the Joy-IT XPLORER board.

The project creates a complete bidirectional control and monitoring system:

- Wokwi Pico W → Remote control panel
- Arduino Nano 33 IoT → MQTT ↔ UART bridge
- Raspberry Pi Pico (XPLORER) → Hardware controller

Commands flow from Wokwi through MQTT and UART to control physical hardware, while sensor readings and hardware state are mirrored back to Wokwi.

---

## System Architecture

```

Wokwi Pico W
(Browser Simulation)

Buttons
Potentiometer
OLED Display
NeoPixel Mirror

↓

MQTT JSON Messages

↓

Arduino Nano 33 IoT

MQTT ↔ UART Bridge

↓

UART (115200 baud)

↓

Raspberry Pi Pico (XPLORER Board)

NeoPixels
Servo
DHT11
Physical Buttons

```

---

## Features

### Wokwi Pico W

- 4 virtual buttons controlling XPLORER LEDs
- Potentiometer controls XPLORER servo
- OLED displays live temperature and humidity
- Mirrors XPLORER NeoPixel state
- MQTT communication with JSON packets
- Sequence number validation
- Replay protection
- Watchdog timeout detection

### Arduino Nano 33 IoT

- MQTT ↔ UART bridge
- JSON validation
- Shared token authentication
- Sequence gap detection
- Replay protection
- Heartbeat LED
- UART watchdog monitoring
- Serial Monitor diagnostics

### XPLORER Raspberry Pi Pico

- Controls 4 WS2812 NeoPixels
- Controls servo motor
- Reads DHT11 sensor
- Handles physical button presses
- UART command parser
- Status reporting
- LED state synchronization

---

## Hardware

### Raspberry Pi Pico (XPLORER)

| Component | GPIO |
|------------|-------|
| NeoPixels | GP1 |
| Servo | GP7 |
| DHT11 | GP22 |
| Top Button | GP10 |
| Right Button | GP11 |
| Bottom Button | GP14 |
| Left Button | GP15 |
| UART TX | GP4 |
| UART RX | GP5 |

### Nano ↔ Pico Wiring

```

Nano 33 IoT          XPLORER Pico

D1 (TX) -----------> GP5 (RX)
D0 (RX) <----------- GP4 (TX)
GND ---------------- GND

```

TX/RX must be cross-connected.

---

## UART Protocol

Commands sent from Nano → Pico:

| Command | Description |
|----------|-------------|
| `LED:N:NEXT` | Advance LED N color |
| `SERVO:angle` | Set servo position |
| `STATUS` | Request full system state |


---

## LED Color Sequence

```

RED
GREEN
BLUE
YELLOW
CYAN
PURPLE
WHITE
OFF

```

---

## UART Responses

Pico → Nano:

| Response | Description |
|-----------|-------------|
| `ACK:LED:N:COLOR` | LED state update |
| `ACK:SERVO:angle` | Servo acknowledgement |
| `TEMP:value` | Temperature reading |
| `HUM:value` | Humidity reading |
| `STATUS:...` | Complete device status |

Example:

```

ACK:LED:1:RED
ACK:SERVO:90
TEMP:26.5
HUM:44
STATUS:LED1=RED,LED2=OFF,SERVO=90,TEMP=26.5,HUM=44

```

---

## MQTT Topics

| Topic | Publisher | Subscriber |
|--------|------------|-------------|
| `iem/task4/xplorer/cmd` | Wokwi Pico W | Nano |
| `iem/task4/xplorer/data` | Nano | Wokwi Pico W |

---

## JSON Format

Wokwi → Nano

```json
{
  "token":"iem2026",
  "source":"pico",
  "seq":0,
  "cmd":"LED:1:NEXT"
}
```

Nano → Wokwi

```json
{
  "token":"iem2026",
  "source":"nano",
  "seq":1,
  "uart":"ACK:LED:1:RED"
}
```

---

## Libraries

### Nano 33 IoT

```cpp
WiFiNINA
PubSubClient
SimpleJson.h
```

### Wokwi Pico W

```cpp
WiFi
PubSubClient
Adafruit NeoPixel
Wire
Adafruit GFX
Adafruit SSD1306
SimpleJson.h
```

### XPLORER Pico

```cpp
Adafruit NeoPixel
Servo
DHT Sensor Library
```

---

## Robustness Features

### Authentication

Shared token validation:

```

iem2026

```

### Sequence Numbers

Detects:

- Lost packets
- Replay attacks
- Out-of-order packets

### Watchdog

Nano:

- UART timeout detection (20 s)

Wokwi:

- MQTT feedback timeout (15 s)

### Heartbeat

Nano onboard LED heartbeat confirms bridge operation.

---

## Project Structure

```

Task4/

├── nano_iot_task4.ino
├── wokwi_pico_task4.ino
├── xplorer_pico.ino
├── SimpleJson.h
└── README.md

```

---

## Usage

### Start Hardware

Upload:

- `nano_iot_task4.ino` → Nano 33 IoT
- `xplorer_pico.ino` → Raspberry Pi Pico
- `wokwi_pico_task4.ino` → Wokwi Pico W

### Test Commands

Serial Monitor:

```

LED:1:NEXT
LED:2:NEXT
SERVO:90
STATUS

```

### Full Chain Test

Wokwi Button

↓

MQTT

↓

Nano

↓

UART

↓

XPLORER LED changes

↓

ACK feedback

↓

Wokwi mirror updates

Potentiometer:

```

Wokwi Potentiometer
→ SERVO command
→ Nano Bridge
→ XPLORER Servo moves

```

Sensor flow:

```

DHT11
→ Pico UART
→ Nano MQTT
→ Wokwi OLED

```

---

## Known Issues/Notes

- XPLORER Pico is the single source of truth.
- Wokwi only mirrors LED state.
- Servo exists physically only on XPLORER.
- DHT11 requires ≥2 second read interval.
- UART uses 115200 baud.
- NeoPixels and servo require external 5V power.

- A work around was needed to stop Wokwi simulator from lagging and dumping messages on Serial on button press
- Neopixel on Wokwi simulator does not work as intended but mirror acknowledgement can be confirmed from Serial and ```CMD:STATUS```

---

## Authors

**Neel Karia (16346352)**

Embedded Lab & Project – Task 4 <br>
UART Bridge + XPLORER Board <br>
Summer Semester 2026

