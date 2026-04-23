# ESP32 Beehive Sensor Node LoRa

Reliable ESP32-based beehive sensor node with HX711, dual DHT22, OLED display, battery monitoring, deep sleep operation and robust LoRa communication.

---

## Overview

This project implements the final firmware for a smart beehive sensor node based on ESP32.

The node is designed for **autonomous field operation**, capable of collecting environmental and structural data from a beehive and transmitting it through a reliable LoRa link.

Main capabilities include:

* hive weight measurement using HX711
* internal and external environmental monitoring using dual DHT22
* local visualization through OLED display
* battery voltage monitoring
* low-power operation using deep sleep cycles
* reliable LoRa communication with ACK and retransmission
* automatic recovery from faults and unstable conditions

This firmware is part of a distributed IoT architecture for smart beehive monitoring.

---

## Features

* ESP32-based embedded system
* HX711 load cell with filtering and calibration
* Dual DHT22 support (internal + external)
* OLED SSD1306 local interface
* Battery voltage monitoring (ADC + calibration)
* Deep sleep low-power operation
* Reliable LoRa communication (SX1276 / RadioLib)
* ACK-based confirmation
* CRC16 integrity validation
* Sequence control
* Retransmission on failure
* Autonomous operation in field conditions
* Battery-powered operation
* Solar-powered validation

---

## Reliability & Fault Tolerance

The node includes mechanisms to ensure stable operation in real-world conditions:

### Boot Watchdog

* monitors initialization time
* forces restart if setup exceeds defined timeout
* prevents lock-up during peripheral initialization

### Persistent Failure Protection

* boot failure counter stored in RTC memory
* detects repeated failed startups
* triggers controlled restart after multiple failures

### Runtime Protection

* monitors execution during active cycle
* detects abnormal delays in loop execution
* prevents system freeze during operation

### Deep Sleep Robustness

* ensures safe transition between wake/sleep cycles
* recovers from unstable states after wake-up

---

## Payload Structure

The transmitted payload contains:

* Internal temperature (°C)
* Internal humidity (%)
* External temperature (°C)
* External humidity (%)
* Hive weight (kg)
* Battery voltage (V)
* Sensor validity flags

All values are transmitted in scaled integer format for efficiency and reliability.

---

## Hardware

### Main Components

* ESP32
* SX1276 LoRa module
* HX711
* Load cell
* 2x DHT22
* OLED SSD1306
* Battery supply
* Solar panel
* Voltage regulator

---

## Pin Mapping

### LoRa

* SCK: GPIO 5
* MISO: GPIO 19
* MOSI: GPIO 27
* CS: GPIO 18
* DIO0: GPIO 26
* RST: GPIO 14

### HX711

* DOUT: GPIO 25
* SCK: GPIO 4

### DHT22

* Internal: GPIO 13
* External: GPIO 17

### OLED

* SDA: GPIO 21
* SCL: GPIO 22
* RST: GPIO 16

### Battery Monitor

* ADC: GPIO 34

---

## Firmware Flow

The node operates in a cyclic low-power model:

1. Wake up from deep sleep
2. Initialize system and peripherals (with watchdog supervision)
3. Read sensors during the active window
4. Capture a stabilized final measurement
5. Read battery voltage
6. Power down HX711 before transmission
7. Transmit payload through LoRa
8. Wait for ACK (with retransmission strategy)
9. Return to deep sleep

---

## LoRa Protocol

A lightweight application-layer protocol is implemented with:

* MAGIC byte
* protocol version
* packet type
* node ID
* sequence number
* payload length
* payload
* CRC16

### Reliability Strategy

* ACK confirmation
* CRC16 integrity validation
* duplicate detection (gateway-side)
* retransmission (up to 3 attempts)

---

## Power Management

* deep sleep cycle: **15 minutes**
* optimized active window
* peripheral shutdown before sleep
* designed for battery + solar operation

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

---

## Current Status

* Stable sensor integration
* HX711 + dual DHT22 + OLED validated
* Reliable LoRa communication validated
* Battery voltage monitoring integrated
* Deep sleep operation validated
* Field operation stability improved (watchdog + recovery)
* Battery-powered validation completed
* Solar-powered validation completed

---

## Next Steps

* long-term field validation (continuous operation)
* adaptive sampling strategies
* multi-node scalability
* advanced diagnostics and telemetry

---

## System Role

This node represents the **data acquisition layer** of the MMS system:

```text
Sensor Node → LoRa → Gateway → Backend → Database → Dashboard
```

---

## Author

Ruan Silveira

Electrical Engineering Student
Industrial Maintenance Technician

Focus areas:

* Embedded Systems
* IoT
* Agro 4.0
* Industrial Automation