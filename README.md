# PenguinLib

Arduino library for controlling legged robots using ESP32.

Developed by **Penguin Lab**, the library provides tools for controlling robot legs, generating movements, working with servo motors through the PCA9685, and integrating sensors and interaction devices.

## Features

* Control of legged robots
* Servo control using the PCA9685
* Leg movement and testing
* Robot poses and movements
* BNO08x IMU integration
* ESP32 support
* Bluetooth control using Dabble
* Touch interaction using ESP32 touch pins
* Audio interaction using DFPlayer Mini

## Hardware

The library is designed primarily for projects based on:

* ESP32
* PCA9685 servo driver
* Servo motors
* SparkFun BNO08x IMU
* DFPlayer Mini

## Dependencies

The following libraries are required (tested versions bellow):

* DabbleESP32 >= 1.5.1
* Adafruit PWM Servo Driver Library >= 3.0.3
* SparkFun BNO08x Cortex Based IMU >= 1.0.6
* PubSubClient >= 2.8.0

These dependencies are installed automatically when PenguinLib is installed through the Arduino Library Manager.

## Examples

The library includes examples demonstrating different levels of functionality.

### 01 - Calibration

Basic servo calibration and configuration.

This example can be used to determine the PWM values corresponding to the desired servo positions.

### 02 - Leg Test

Tests the individual joints and movements of a robot leg.

Useful for verifying servo connections, directions, limits, and mechanical assembly.

### 03 - Hexapod

Demonstrates basic control of a complete hexapod robot.

It combines the individual legs to generate coordinated robot movements.

### 04 - Hexapod Reaction with BNO

Demonstrates the use of the BNO08x IMU to make the hexapod react to its orientation.

The robot can use information from the IMU to detect changes in its orientation and respond accordingly.

### 05 - Petting and Audio

Demonstrates an interactive robot using:

* ESP32 touch pins
* DFPlayer Mini
* Audio feedback
* Touch-based interaction for petting

The robot can detect interaction through its touch sensor and respond with movement and audio.

### 06 - Hexapod controlled using MQTT protocol

Demonstrates basic control of a complete hexapod robot using MQTT protocol instead of Dabble app.

## Installation

### Arduino Library Manager

Open the Arduino IDE and go to:

**Sketch → Include Library → Manage Libraries...**

Search for:

```text
PenguinLib
```

Click **Install**.

The required dependencies will be installed automatically.

### Manual Installation

Alternatively, download or clone this repository into your Arduino libraries directory:

```text
Documents/Arduino/libraries/PenguinLib
```

Restart the Arduino IDE after installation.

## Basic Usage

See the examples included with the library for complete applications.

## Project

PenguinLib is being developed as part of the **Penguin Lab** robotics projects, with a focus on making the development of legged robots more accessible and modular.

## License

This project is licensed under the terms of the license included in this repository.
