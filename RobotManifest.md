# Paddock Pal Robot Engineering Document

## Overview

The **Paddock Pal Robot** is designed to automate the process of mucking stalls within donkey paddocks at an animal shelter. The robot must navigate an unstructured and dynamic outdoor environment, detect and avoid temporary and permanent obstacles, and perform physical tasks such as arm actuation for mucking. This requires integration of several sensing and control systems: **LiDAR**, **GPS**, and a **Camera system**, all orchestrated by a central **Control System**.

---

## System Goals

- **Autonomous Stall Cleaning**: Efficiently clean stalls without human intervention.
- **Environmental Awareness**: Detect boundaries, obstacles, and objects of interest.
- **Precise Navigation**: Utilize GPS and local sensing to follow a path and avoid hazards.
- **Modular Software**: Divide logic into subsystems to allow for testing, scalability, and upgradeability.

---

## Core Sensors and Subsystems

### 1. GPS Subsystem

**Purpose**:
- To provide global positioning data that defines the robot’s position within the paddock.
- To map out the paddock boundaries for navigation constraints.
- To relay positional data to the control system for global grid reference alignment.

**Data Flow**:  
`Raw GPS → Noise filtering / smoothing → Boundary awareness → Position update to control system`

**Considerations**:
- Must account for GPS drift; sensor fusion with IMU may be considered in future iterations.
- Works best in open-air environments typical of paddocks.

---

### 2. LiDAR Subsystem

**Purpose**:
- To perform real-time detection of temporary or dynamic obstacles (e.g., animals, tools, waste).
- To generate offset vectors that guide reactive avoidance behavior.
- To prevent collisions by triggering emergency stop signals if a path is blocked.

**Data Flow**:  
`Raw point cloud → Obstacle segmentation → Collision prediction → Command to control system`

**Features**:
- Short-range obstacle detection in 360 degrees (depending on unit).
- Integration with control logic to modify or halt movement commands.

---

### 3. Camera Subsystem

**Purpose**:
- To identify mucking targets (waste piles) through vision processing and maneuver alignment.
- To assist with object recognition and classification (e.g., determining waste vs. animal).
- To control actuation of the robotic arm responsible for mucking.

**Data Flow**:  
`Camera frame → Computer vision algorithm → Waste detection → Arm offset + signal to pause → Control system update`

**Capabilities**:
- Frame-by-frame inference using object detection (YOLO, SSD, etc.).
- Visual cues for aligning mucking tools.
- Optional human/animal presence alerting.

---

## Control System

**Overview**:  
The control system serves as the central hub that integrates and interprets data from the GPS, LiDAR, and Camera subsystems. It also runs the **pathing algorithm** and translates movement plans into motor commands, ultimately driving the robot.

**Responsibilities**:
- Fusion of sensor data into a coherent world model (grid-based).
- Management of robot pose `(x, y, θ)` using GPS and odometry.
- Execution of **A*** path planning based on gridded map and dynamic inputs.
- Translating waypoints into velocity and direction vectors.
- Issuing pinout commands to the Jetson Nano (or equivalent) for motor control and arm actuation.

**Architecture**:
- **Subsystem Input Layer**: Receives structured messages (e.g., ROS2 topics or custom messaging format)
- **Mapping & Localization Layer**: Maintains occupancy grid and updates with incoming sensor data
- **Planning Layer**: Runs A* algorithm to determine shortest valid path to next target
- **Motor Control Layer**: Converts waypoints into movement primitives (turn, forward, stop)
- **Actuation Layer**: Sends GPIO/PWM/I2C/SPI signals to motor drivers and actuators

---

## Software Architecture

### Proposed ROS2 Node Structure

| Node Name            | Responsibility                          | Inputs                        | Outputs                       |
|----------------------|-----------------------------------------|-------------------------------|-------------------------------|
| `gps_node`           | Read and process GPS data               | GPS Module                    | Current position              |
| `lidar_node`         | Process LiDAR point clouds              | LiDAR Sensor                  | Obstacle alerts, offset       |
| `camera_node`        | Detect mucking targets                  | USB/CSI Camera                | Waste detected, offsets       |
| `control_node`       | Core logic, grid mapping, pathing       | Inputs from all subsystems    | Movement & actuator commands  |
| `motor_driver_node`  | Translate movement to pinouts           | Movement commands             | GPIO/PWM signals              |
| `arm_control_node`   | Raise/lower and actuate mucking arm     | Camera signals / Control Cmds | Arm state                     |

---

## Path Planning

The control system converts the real-world environment into a **discretized grid**. A* pathfinding is implemented over this grid to plan the most efficient route to the target mucking point. The robot must dynamically replan when obstacles are detected by LiDAR.

- **Inputs**: Start/goal positions, occupancy grid
- **Outputs**: Sequence of waypoints
- **Constraints**: Avoid dynamic obstacles, respect paddock boundaries

---

## Communications and Data Flow

- All sensors communicate asynchronously with the control system.
- Data is time-stamped and stored in a circular buffer for state estimation and debugging.
- ROS2 services and topics are used to maintain modularity and traceability.

---

## Next Steps

1. Finalize sensor selection and calibration specs.
2. Develop simulation environment to test pathing and control.
3. Define message schema between subsystems (ROS2 messages or custom).
4. Build testing framework for obstacle detection and recovery behaviors.
5. Begin hardware integration and bring-up phase on the Jetson.

