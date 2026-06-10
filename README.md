# Secure Automated Dispensing System

An embedded dispensing platform designed to provide secure and automated liquid dispensing through password authentication, liquid-level monitoring, cup detection, and wireless status reporting.

## Overview

This project was developed using the PIC18F45K22 microcontroller and combines sensing, authentication, actuation, and wireless communication into a complete embedded system.

The system authenticates users through a keypad interface, verifies cup placement, monitors liquid levels using an ultrasonic sensor, controls dispensing through a motor-driven pump, and reports system status wirelessly to a remote monitoring unit.

## Key Features

* Password authentication
* Cup detection
* Ultrasonic liquid-level monitoring
* Automated dispensing control
* Wireless RF status reporting
* LCD user interface
* Interrupt-driven sensing and control

## System Architecture

Keypad
   ↓
Authentication
   ↓
Cup Detection
   ↓
Liquid Monitoring
   ↓
Pump Control
   ↓
RF Communication

## Hardware Components

### Transmitter Unit

* PIC18F45K22
* 3×4 Keypad
* HC-SR04 Ultrasonic Sensor
* Infrared Sensors
* LCD Display
* Motor Driver
* DC Pump
* HT12E Encoder
* FS100A RF Transmitter

### Receiver Unit

* PIC18F45K22
* HT12D Decoder
* FS100A RF Receiver
* LCD Display

## Engineering Contributions

* Embedded firmware development in C
* Password authentication implementation
* LCD driver development
* Sensor integration
* Interrupt-based event handling
* Wireless RF communication
* Actuator control
* System integration and testing

## Demonstration

Project Video:

https://www.youtube.com/watch?v=DcZ19SxXLVQ

## Documentation

The repository includes:

* Firmware source code
* System block diagrams
* Circuit schematics
* Demonstration video

## Future Improvements

* Wireless telemetry logging
* Mobile application integration
* Enhanced user management
* Real-time monitoring dashboard
