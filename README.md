# Autonomous Mecanum-Wheel Robot — Sparse SLAM, Simulation & Embedded Systems

An autonomous omnidirectional mobile robot that maps and navigates unknown indoor environments using only 5 time-of-flight sensors. The project includes a full Gazebo simulation, a custom C++ SLAM and navigation stack running under ROS 2 Humble, and ESP-IDF firmware with micro-ROS integration for the physical ESP32-C3 platform.

Originally built for the MEAM 5100 Mechatronics course at the University of Pennsylvania, the robot has been extended into a complete autonomous exploration platform with sim-to-real capability.

## System Architecture

```
Physical Robot (ESP32-C3)                    PC (WSL2 / Ubuntu 22.04)
┌──────────────────────────┐                ┌─────────────────────────────┐
│  5x VL53L0X ToF sensors  │                │  micro-ROS Agent (UDP)      │
│  LSM6DS3TR-C + LIS3MDL   │   WiFi/UDP    │         │                   │
│  Madgwick filter (100Hz) │ ─────────────► │  EKF (robot_localization)   │
│  Wheel encoders          │                │         │                   │
│  4x DC motors (mecanum)  │                │  Sparse SLAM (occupancy     │
│  micro-ROS publishers    │                │    grid from 5 ToF beams)   │
│                          │ ◄───────────── │         │                   │
│  cmd_vel subscriber      │                │  Navigator (frontier        │
└──────────────────────────┘                │    exploration + A*)        │
                                            │         │                   │
                                            │  RViz2 (visualization)      │
                                            └─────────────────────────────┘
```

## Hardware

- **Chassis:** 3-tier laser-cut acrylic (8" x 10" footprint), custom 3D-printed motor mounts and sensor brackets, designed in SolidWorks
- **Drivetrain:** 4 mecanum wheels with independent DC motors for omnidirectional movement (forward, strafe, rotation)
- **Main Controller:** ESP32-C3 (M5Stamp C3, RISC-V, 160MHz)
- **Motor Controller:** ItsyBitsy ATmega32U4, communicating via I2C
- **Sensors:**
  - 5x VL53L0X time-of-flight ranging sensors (I2C, address-sequenced via D flip-flop shift register)
  - Adafruit LSM6DS3TR-C + LIS3MDL 9-DOF IMU with onboard Madgwick orientation filter
  - Wheel encoders for dead reckoning
- **Electronics:** All circuits hand-soldered on perfboard with Molex/XT60 connectors, custom SN754410 H-bridge motor driver with AND/NOR gate complementary PWM generation

## Repository Structure

```
mecanum_ws/
├── src/
│   └── mecanum_robot/              # ROS 2 package (simulation + navigation)
│       ├── src/
│       │   ├── sparse_slam.cpp     # Occupancy grid SLAM from 5 ToF sensors
│       │   └── navigator.cpp       # Frontier exploration + A* path planning
│       ├── urdf/
│       │   └── mecanum_robot.urdf.xacro
│       ├── meshes/                 # SolidWorks STL exports
│       ├── launch/
│       │   ├── display.launch.py   # RViz visualization
│       │   └── gazebo.launch.py    # Full simulation launch
│       ├── config/
│       │   └── ekf.yaml            # EKF configuration
│       └── scripts/
│           ├── imu_relay.py        # Adds covariance to IMU messages
│           ├── twist_relay.py      # Remaps velocity axes
│           └── center_mesh.py      # Utility to center STL origins
│
└── firmware/                       # ESP-IDF firmware for ESP32-C3
    ├── main/
    │   ├── main_controller.cpp     # Robot controller with Madgwick filter
    │   └── libraries/              # Adafruit VL53L0X, BusIO, Sensor
    ├── components/
    │   ├── arduino/                # Arduino as ESP-IDF component
    │   └── micro_ros_espidf_component/
    ├── CMakeLists.txt
    └── sdkconfig.defaults
```

## Simulation

The Gazebo simulation replicates the physical robot with:

- Custom URDF/xacro model with SolidWorks mesh imports
- 5 Gazebo ray sensor plugins simulating VL53L0X ToF sensors with 25° FoV cone (5-ray fan)
- IMU sensor plugin with Gaussian noise
- Planar move plugin for mecanum omnidirectional kinematics
- Joint state publisher for wheel visualization
- EKF (robot_localization) fusing odometry and IMU

### Running the Simulation

```bash
# Terminal 1: Launch everything (builds, sources, and starts Gazebo + all nodes)
source /opt/ros/humble/setup.bash
cd ~/mecanum_ws
colcon build --base-paths src
source install/setup.bash
ros2 launch mecanum_robot gazebo.launch.py

# Terminal 2: Visualize in RViz
rviz2 --ros-args -p use_sim_time:=true
# Set Fixed Frame to "odom", add RobotModel and /map OccupancyGrid
```

The launch file starts Gazebo, robot_state_publisher, EKF, sparse SLAM, IMU relay, twist relay, and the autonomous navigator — all in one command.

## SLAM

The sparse SLAM system builds occupancy grid maps using only 5 ToF sensors:

- **Occupancy grid:** 400x400 cells at 5cm resolution (20m x 20m coverage)
- **Log-odds updates:** Bresenham raycasting marks free cells along each beam, occupied cells at hit points
- **Max-range handling:** Rays that don't hit anything trace 2m of free space without marking an endpoint
- **Pose source:** EKF-fused odometry + IMU via `/odometry/filtered`

The two-mode exploration architecture:

1. **Explore mode:** Robot drives toward frontiers while all 5 ToF sensors continuously update the map. Pure dead reckoning + IMU for pose tracking.
2. **Refine mode:** When accumulated travel distance exceeds a threshold, the robot stops and performs a heading-based 360° spin, collecting dense range data from all sensors. This produces a pseudo-lidar scan for map refinement and (future) scan matching.

## Navigation

Autonomous frontier exploration with:

- **Frontier detection:** Flood-fill clustering of free cells adjacent to unknown cells
- **Target selection:** Nearest frontier cell beyond a minimum distance threshold
- **A\* path planning:** 8-connected grid search on the inflated occupancy grid with obstacle inflation for safety margins
- **Line-of-sight path smoothing:** Bresenham-based visibility checks eliminate unnecessary waypoints for straight-line driving in open space
- **Heading control:** Rotate-then-drive with proportional heading correction, accounting for the robot's +Y forward axis
- **Reactive obstacle avoidance:** Immediate stop and rescan if any ToF reading drops below 15cm during driving

## Sensor Fusion

Extended Kalman Filter (robot_localization) fusing:

| Source | Provides | Update Rate |
|--------|----------|-------------|
| Wheel odometry | x, y position; vx, vy velocity; yaw rate | 50 Hz |
| IMU | yaw orientation; yaw rate | 100 Hz |

The EKF runs in 2D mode, publishing the corrected `odom → base_link` transform and `/odometry/filtered` odometry.

## Firmware

The ESP32-C3 firmware is built with ESP-IDF v5.3 using Arduino as a component:

- **Cooperative scheduler:** Timed handler functions at configurable intervals (10ms IMU, 25ms dead reckoning, 100ms lasers, 100ms navigation)
- **Madgwick orientation filter:** 6DOF (accel + gyro) running at 100Hz, outputting quaternion orientation. Direct I2C register access bypassing Adafruit library for ESP-IDF compatibility.
- **ToF management:** D flip-flop shift register sequences I2C addresses for 5 sensors sharing the same bus
- **Motor control:** I2C commands to ItsyBitsy motor controller with mecanum kinematic mixing
- **micro-ROS ready:** ESP-IDF component linked, publishers to be added for wireless ROS 2 communication

### Building the Firmware

```bash
# Use a separate terminal from ROS — do not source both
. ~/esp/esp-idf/export.sh
cd ~/mecanum_ws/firmware
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Future Work

- **micro-ROS publishers:** Stream ToF, IMU, and odometry data over WiFi to the PC-side SLAM stack
- **ICTE scan matching:** Implementation of Filotheou's correspondenceless scan-to-map matching algorithm for pose correction using the spin-scan data
- **Magnetometer calibration:** Hard-iron offset characterization for absolute heading from the LIS3MDL
- **Odometry noise injection:** Gaussian noise relay for realistic simulation testing
- **Docker:** Containerized simulation stack with docker-compose for reproducible builds
- **Map saving/loading:** Serialize occupancy grid for multi-session navigation

## Dependencies

### Simulation (Ubuntu 22.04 / WSL2)
- ROS 2 Humble
- Gazebo Classic 11
- robot_localization
- xacro
- joint_state_publisher_gui

### Firmware
- ESP-IDF v5.3.2
- Arduino-ESP32 v3.1.1 (as ESP-IDF component)
- micro-ROS ESP-IDF component (iron branch)
- Adafruit VL53L0X library
- Adafruit BusIO library

## Author

Built by Mark Williams — University of Pennsylvania, MEAM 5100 Mechatronics.
