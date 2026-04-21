# ESP32 Beehive Sensor Node LoRa

Reliable ESP32 beehive sensor node with HX711, dual DHT22, OLED, battery monitoring, deep sleep and LoRa communication.

---

## Overview

This project implements the final firmware for a smart beehive sensor node based on ESP32.

The node is capable of:

- reading hive weight through HX711
- reading internal and external environmental data using two DHT22 sensors
- displaying local status on an OLED display
- monitoring battery voltage
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
- Battery voltage monitoring
- Deep sleep low-power operation
- Reliable LoRa communication
- ACK-based confirmation
- CRC16 integrity check
- Sequence control
- Retransmission on failure
- Battery-powered operation
- Solar-powered field validation

---

## Payload Structure

The transmitted payload contains:

- Internal temperature (°C)
- Internal humidity (%)
- External temperature (°C)
- External humidity (%)
- Hive weight (kg)
- Battery voltage (V)
- Validity flags for sensor readings

All values are sent in scaled integer format for efficient transmission.

---

## Hardware

### Main Components

- ESP32
- SX1276 LoRa module
- HX711
- Load cell
- 2x DHT22
- OLED SSD1306
- Battery supply
- Solar panel
- Voltage regulator

### Pin Mapping

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

#### Battery Monitor

- ADC: GPIO 34

---

## Firmware Flow

The node operates in a cyclic low-power model:

1. Wake up from deep sleep
2. Initialize peripherals
3. Read sensors during the active window
4. Capture a final snapshot
5. Read battery voltage
6. Power down HX711 before transmission
7. Send payload through reliable LoRa
8. Wait for ACK / retry if needed
9. Return to deep sleep

---

## LoRa Protocol

A lightweight application-layer protocol is used with the following fields:

- MAGIC byte
- protocol version
- packet type
- node ID
- sequence number
- payload length
- payload
- CRC16

### Reliability Strategy

- ACK confirmation
- CRC16 integrity check
- duplicate protection on gateway side
- retransmission up to 3 attempts

---

## Example Transmitted Data

- Weight
- Battery voltage
- Internal temperature and humidity
- External temperature and humidity

---

## Project Structure

```text
include/
lib/
src/
test/
platformio.ini
README.md
```

## Current Status

- Stable sensor integration
- Reliable LoRa              communication validated
- HX711 + dual DHT22 + OLED integrated
- Battery voltage monitoring integrated
- Deep sleep working
- Battery-powered validation completed
- Solar-powered validation - completed

## Next Steps

- Gateway to backend integration
- Web dashboard synchronization
- Long-term field deployment
- Multiple hive support

## Author

Ruan Silveira

Electrical Engineering Student
Industrial Maintenance Technician

Focus areas:

- Embedded Systems
- IoT
- Agro 4.0
- Industrial Automation