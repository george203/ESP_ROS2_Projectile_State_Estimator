# CLAUDE.md

## Project Overview
ROS2 IMU sensor fusion system. An ESP32-S3 reads MPU-6050 accelerometer/gyroscope data over I2C and publishes it as ROS2 topics via micro-ROS over WiFi. A C++ ROS2 node on the host machine applies a complementary filter for sensor fusion and outputs filtered orientation. Visualized in RViz2.

This is a portfolio project targeting embedded software engineer roles at AV/robotics companies (Waymo, Zoox, etc.).

## Hardware
- **MCU:** ESP32-S3 DevKitC-1-N8R8
- **IMU:** MPU-6050 GY-521
- **I2C wiring:** SDA → GPIO 8, SCL → GPIO 9, VCC → 3.3V, GND → GND
- **MPU-6050 I2C address:** 0x68

## Repository Structure
```
├── CLAUDE.md
├── firmware/          # ESP-IDF project for ESP32-S3
│   ├── main/
│   │   └── main.c     # micro-ROS publisher + MPU-6050 I2C driver
│   ├── CMakeLists.txt
│   └── sdkconfig
└── ros2_ws/           # ROS2 Jazzy workspace on host machine
    └── src/
        └── imu_fusion/
            ├── src/
            │   └── fusion_node.cpp   # Complementary filter + publisher
            ├── CMakeLists.txt
            └── package.xml
```

## Firmware (ESP-IDF / micro-ROS)
- Written in C using ESP-IDF v5.x
- Uses micro-ROS ESP-IDF component (`micro-ROS/micro_ros_espidf_component`)
- Transport: micro-ROS over WiFi UDP to host machine
- Publishes: `sensor_msgs/msg/Imu` on topic `/imu/raw`
- MPU-6050 driver reads registers directly (no library) — see datasheet for `ACCEL_XOUT_H` and `GYRO_XOUT_H` register map
- Scale factors: apply MPU-6050 datasheet conversion to get m/s² and rad/s from raw ADC counts

## ROS2 (Host Machine)
- ROS2 Jazzy on Ubuntu
- `imu_fusion` package: C++ node subscribing to `/imu/raw`, publishing filtered orientation to `/imu/filtered`
- Sensor fusion: complementary filter combining accelerometer (low-freq) and gyroscope (high-freq)
- Visualization: RViz2

## Key Constraints
- Do not use external MPU-6050 libraries in the firmware — register reads should be written from the datasheet. This is intentional for learning and interview purposes.
- Keep micro-ROS memory configuration conservative — the S3 has 512KB internal SRAM; micro-ROS allocations should stay in internal RAM.
- ROS2 node should be written in C++, not Python.

## Developer Background
- Familiar with ESP-IDF, I2C, GPIO, PWM from prior ESP32 projects
- Learning ROS2 and micro-ROS for the first time on this project
- Target audience for code review: embedded SW hiring managers at AV companies
