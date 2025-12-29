# Autonomous Vision-Based Mobile Robot  
### Object Localization and Transport using ESP32-S3

# ! TO DO !

## About the Project 

This repository contains the firmware for an **autonomous mobile robot** designed to **detect, track, and transport a physical object**. Object detection is performed using an onboard neural network, while robot behavior is governed by a **finite state machine (FSM)**.

The robot is built around an **ESP32-S3 microcontroller with a camera module** and integrates multiple sensors, including wheel encoders, an IMU, and a time-of-flight distance sensor.  


---

## Key Features

- Vision-based object detection and localization
- Autonomous navigation toward detected objects
- Object capture and return to a predefined base position
- Finite State Machine–based control architecture
- Sensor fusion using encoders, IMU, and ToF distance sensor
- Optional camera streaming and onboard data logging

---

## System Overview

Camera → Object Detection (NN) → FSM Decision Logic → Motor Control

**Sensors and peripherals:**
- Camera (RGB565, 240×240)
- Wheel encoders (differential drive)
- IMU (MPU6050)
- ToF distance sensor (VL53 series)
- DC motors with TB6612FNG H-bridge 
- Onboard data logging (LittleFS)

---

## Finite State Machine (FSM)

The robot behavior is implemented as a finite state machine with the following states:

- **IDLE** – system waiting for a valid object detection  
- **CHASE** – tracking and approaching the detected object  
- **CAPTURED** – object successfully reached and secured  
- **RETURN** – returning to the base position with the object  
- **HOME** – object delivered to the base position  

State transitions depend on detection confidence, sensor feedback, and timing constraints.


## Data Logging and Debugging

The system supports onboard data logging using LittleFS:
- Encoder pulses and velocities
- IMU acceleration and angular velocity
- Logged at 20 Hz
- Manual data save using BOOT button

Serial debug output provides FSM state transitions and diagnostic information.

---

## Configuration Flags

Key compile-time options:

```cpp
#define ENABLE_PERIPHERALS 1
#define ENABLE_STREAM      0
#define ENABLE_SAVING_DATA 1
```

ENABLE_STREAM – enables Wi-Fi camera streaming (performance-heavy)

ENABLE_SAVING_DATA – enables onboard data logging

ENABLE_PERIPHERALS – enables motors and sensors



```cpp
ESP32-TEST/
├── include/ # Board/camera configuration headers
│ ├── board_config.h
│ ├── camera_index.h
│ └── camera_pins.h
├── lib/ # Project modules (libraries)
│ ├── DataLogger/
│ ├── Detection/
│ ├── Encoders/
│ ├── Fsm/
│ ├── ImuMPU/
│ ├── LogConsole/
│ ├── Motors/
│ ├── Servo360/
│ ├── Tennis-Ball-detection_inferencing/
│ └── TofVL53/
├── src/ # Application sources
│ ├── app_httpd.cpp # Camera server (when streaming enabled)
│ └── main.cpp      # Main firmware entry point
├── test/
├── partitions.csv  # Flash partition table
├── platformio.ini
└── README.md
```


## Build & Flash (PlatformIO)

### Requirements
- VS Code + PlatformIO extension  

### Build
```bash
pio run
```

### Upload
```bash
pio run -t upload
```