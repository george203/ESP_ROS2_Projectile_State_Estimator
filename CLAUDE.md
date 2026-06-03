# CLAUDE.md

## Project Overview
ROS2 IMU-only inertial state estimation system. An ESP32-S3 reads MPU-6050 accelerometer/gyroscope data over I2C and publishes it as ROS2 topics via micro-ROS over WiFi. A C++ ROS2 node on the host machine runs an Extended Kalman Filter (EKF) to estimate position, velocity, and orientation during short ballistic flights of the device. A separate node computes the physics-predicted ground-truth projectile trajectory from inferred initial conditions for comparison. Both trajectories are visualized in RViz2, with covariance ellipsoids showing the EKF's growing uncertainty during the unmeasured flight phase.

The demonstration: launch the device (toss with the breadboard attached to a soft mount, or swing on a pendulum, or slide across a flat surface), let it move, then bring it back to rest. The EKF tracks the trajectory using only IMU integration. Visualizing the estimate against the physics ground truth illustrates the inherent drift of IMU dead reckoning — the same problem that motivates GPS, visual-inertial odometry, and other multi-sensor fusion strategies in autonomous vehicles.

This is a portfolio project targeting embedded software engineer roles at AV/robotics companies (Waymo, Zoox, etc.). The chosen problem — IMU-only state estimation during GPS-denied or sensor-dropout scenarios — is directly relevant to AV redundancy systems.

## Hardware
- **MCU:** ESP32-S3 DevKitC-1-N8R8
- **IMU:** MPU-6050 GY-521 (3-axis accelerometer + 3-axis gyroscope)
- **I2C wiring:** SDA → GPIO 8, SCL → GPIO 9, VCC → 3.3V, GND → GND
- **MPU-6050 I2C address:** 0x68
- **Strap pin avoidance:** No peripherals on GPIO 0, 3, 45, 46 (sampled at boot to determine boot mode; stray pulls block flashing)

## Repository Structure
```
├── CLAUDE.md
├── README.md
├── .gitignore
├── firmware/
│   └── imu_node/                       # ESP-IDF project for ESP32-S3
│       ├── main/
│       │   ├── imu_node.c              # micro-ROS publisher + MPU-6050 I2C driver
│       │   └── CMakeLists.txt
│       ├── components/
│       │   └── micro_ros_espidf_component/
│       ├── CMakeLists.txt
│       └── sdkconfig
└── ros2/                               # ROS2 Jazzy workspace on host machine
    └── src/
        ├── imu_ekf/                    # EKF state estimator node
        │   ├── src/
        │   │   ├── ekf_node.cpp
        │   │   └── ekf.cpp
        │   ├── include/imu_ekf/
        │   │   └── ekf.hpp
        │   ├── CMakeLists.txt
        │   └── package.xml
        ├── ideal_trajectory/           # Physics-based trajectory predictor
        │   ├── src/
        │   │   └── ideal_trajectory_node.cpp
        │   ├── CMakeLists.txt
        │   └── package.xml
        └── flight_visualization/       # RViz config + launch files
            ├── launch/
            │   └── flight.launch.py
            ├── rviz/
            │   └── flight.rviz
            └── package.xml
```

## Firmware (ESP-IDF / micro-ROS)
- Written in C using ESP-IDF v5.3
- Uses micro-ROS ESP-IDF component (`micro-ROS/micro_ros_espidf_component`, `jazzy` branch)
- Transport: micro-ROS over WiFi UDP to host machine
- Publishes: `sensor_msgs/msg/Imu` on topic `/imu/raw` at 100 Hz
- MPU-6050 driver reads registers directly from the datasheet (no external library)
- Scale factors apply MPU-6050 datasheet conversion: raw int16 → m/s² and rad/s
- Covariance fields populated from datasheet noise density specifications (not left at zero)
- Time synchronization via `rmw_uros_sync_session()` for valid wall-clock timestamps

## ROS2 Nodes (Host Machine)
- ROS2 Jazzy on Ubuntu 24.04 ARM64 (running in UTM VM on Apple Silicon)
- Workspace at `ros2/` (build with `colcon build --symlink-install` from that directory)
- All nodes implemented in C++ (Eigen used for matrix math)

### `imu_ekf` — Extended Kalman Filter state estimator
- **Subscribes**: `/imu/raw` (`sensor_msgs/msg/Imu`)
- **Publishes**:
  - `/state_estimate` (`nav_msgs/msg/Odometry`) at 100 Hz — full state with covariance
  - `/path_estimated` (`nav_msgs/msg/Path`) — accumulated trail for RViz
  - `/launch_event` (custom msg) — fires when launch detected
- **Broadcasts TF**: `world` → `imu_link` (from estimated pose)
- **State vector (16-D)**:
  - position (3): x, y, z in world frame
  - velocity (3): vx, vy, vz in world frame
  - orientation (4): quaternion world ← body
  - accelerometer bias (3): bax, bay, baz
  - gyroscope bias (3): bgx, bgy, bgz
- **Process model**: IMU strapdown integration (accel and gyro drive state forward)
- **Measurement updates**:
  - Zero Velocity Update (ZUPT) when stationary (pre-launch, post-landing): asserts velocity = 0
  - Gravity-vector update when stationary: corrects orientation drift using known gravity direction
  - No updates during flight (pure prediction, covariance grows monotonically)
- **Initialization**: ~5 seconds of stationary readings averaged for bias estimates and initial orientation

#### Implementation status
- `ekf.hpp` / `ekf.cpp`: predict step complete — 16-state strapdown integration, full F Jacobian, covariance propagation
- `ekf_node.cpp`: subscribes to `/imu/raw`, calls `ekf_.predict()` on each message
- Not yet implemented: publisher for `/state_estimate`, ZUPT, gravity update, launch/landing detection, path publishing, TF broadcast

### `ideal_trajectory` — Physics ground-truth predictor
- **Subscribes**: `/launch_event` (custom msg with launch timestamp + initial conditions from EKF)
- **Publishes**: `/path_truth` (`nav_msgs/msg/Path`)
- Computes ideal ballistic trajectory: position(t) = p₀ + v₀·t + ½·g·t²
- Initial position and velocity taken from EKF state at launch detection moment
- Provides a reference against which to measure EKF drift

### `flight_visualization` — RViz preset and launch
- RViz2 config with both paths displayed in distinct colors (e.g., green for truth, red for estimate)
- Covariance ellipsoids drawn at intervals along the estimated path (visualizes growing uncertainty)
- TF triad showing live estimated orientation
- Single launch file to bring up all three nodes plus RViz

## Flight Phases and Event Detection
1. **Pre-launch (stationary)**: EKF in calibration mode. Bias estimation active. ZUPT and gravity updates running. Position fixed at origin.
2. **Launch detection**: Linear acceleration magnitude crosses threshold (e.g., > 3g sustained for 50 ms). Capture state as initial condition. Switch to flight mode.
3. **Flight phase**: Pure prediction. No measurement updates. Covariance grows. EKF and physics-truth paths begin to diverge — that divergence is the demonstration.
4. **Landing detection**: Deceleration spike + return to stationary readings. Switch back to update mode.
5. **Post-landing**: ZUPT and gravity updates resume. EKF "snaps back" toward consistency with reality, visualizing the value of measurement updates.

## Key Constraints
- **No external MPU-6050 libraries in firmware** — register reads written from datasheet (interview talking point: "I can explain every line").
- **No external Kalman filter libraries** — EKF implemented from textbook math using Eigen for linear algebra only. Demonstrates understanding of the algorithm, not just usage.
- Conservative micro-ROS memory configuration — ESP32-S3 has 512 KB internal SRAM.
- ROS2 nodes in C++ (not Python). Eigen for matrix math; no ROS2 filter libraries.
- The project must honestly demonstrate drift — that's the point. Polished demos that hide the limitation miss the engineering value.

## Developer Background
- Familiar with ESP-IDF, I2C, GPIO, PWM from prior ESP32 projects
- Learning ROS2 and micro-ROS for the first time on this project
- New to Kalman filtering — plan to implement EKF from textbook math, not from filter libraries
- Target audience for code review: embedded SW hiring managers at AV companies