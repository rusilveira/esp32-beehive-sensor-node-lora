# ESP32 Beehive Sensor Node LoRa

Reliable ESP32 beehive sensor node with HX711, dual DHT22, OLED, deep sleep and LoRa communication.

## Overview

This project implements the final firmware for a smart beehive sensor node based on ESP32.

The node is capable of:

- reading weight data through HX711
- reading internal and external environmental data using two DHT22 sensors
- displaying local status on an OLED display
- operating in low-power cycles using deep sleep
- transmitting data through LoRa (SX1276)
- using a lightweight reliable protocol with ACK, CRC16, sequence number and retransmission

This firmware is part of a distributed IoT architecture for smart beehive monitoring.

---

## Features

- ESP32-based sensor node
- HX711 load cell reading with filtering
- Dual DHT22 support
- OLED SSD1306 local display
- Deep sleep low-power operation
- Reliable LoRa communication
- ACK-based confirmation
- CRC16 integrity check
- Sequence control
- Retransmission on failure
- Battery-powered operation
- Solar-powered field validation

---

## Hardware

### Main components

- ESP32
- SX1276 LoRa module
- HX711
- Load cell
- 2x DHT22
- OLED SSD1306
- Battery supply
- Solar panel
- Voltage regulator

### Pin map

#### LoRa
- SCK: GPIO 5
- MISO: GPIO 19
- MOSI: GPIO 27
- CS: GPIO 18
- DIO0: GPIO 26
- RST: GPIO 14

#### HX711
- DOUT: GPIO 25
- SCK: GPIO 4

#### DHT22
- Internal: GPIO 13
- External: GPIO 17

#### OLED
- SDA: GPIO 21
- SCL: GPIO 22
- RST: GPIO 16

---

## Firmware Flow

The node operates in a cyclic low-power model:

1. Wake up from deep sleep
2. Initialize peripherals
3. Read sensors during the active window
4. Capture a final snapshot
5. Power down HX711 before transmission
6. Send payload through reliable LoRa
7. Wait for ACK / retry if needed
8. Return to deep sleep

---

## LoRa Protocol

A lightweight application-layer protocol is used with the following mechanisms:

- MAGIC byte
- protocol version
- packet type
- node ID
- sequence number
- payload length
- payload
- CRC16

### Reliability strategy

- ACK confirmation
- CRC16 integrity check
- duplicate detection
- retransmission up to 3 attempts

---

## Project Structure

```text
include/
src/
lib/
test/
docs/
platformio.ini
README.md
```

## Current Status

- Stable sensor integration
- Reliable LoRa communication validated
- HX711 + DHT22 + OLED integrated
- Deep sleep working
- Battery-powered validation completed
- Solar-powered validation completed

## Future Improvements

- battery voltage monitoring
- gateway to backend integration
- web dashboard synchronization
- long-term field deployment
- multiple hive support

## Author

Ruan Silveira
Electrical Engineering Student
Industrial Maintenance Technician

Focus areas:

- Embedded Systems
- IoT
- Agro 4.0
- Industrial Automation

