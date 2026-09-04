# ROS2 ICP + EKF Localization for AMR

A ROS2-based localization project implementing **2D LiDAR ICP localization and EKF sensor fusion** for an Autonomous Mobile Robot (AMR).

The objective of this project is to build the localization pipeline from the ground up using **C++ and ROS2**, rather than relying only on existing localization packages.

## 1. Project Overview

This project develops an AMR localization system using:

- 2D LiDAR
- Wheel Odometry
- IMU
- ICP Scan Matching
- Extended Kalman Filter (EKF)

The final localization pipeline will estimate the robot pose:

```text
[x, y, yaw]
```

by combining LiDAR-based global localization with wheel odometry and IMU measurements.

The project is developed and tested in a Gazebo simulation environment before evaluating localization accuracy against simulation ground truth.

---

## 2. Development Environment

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS2 Humble |
| Language | C++ |
| Simulation | Gazebo Fortress |
| Visualization | RViz2 |
| Build System | colcon / CMake |
| Robot Model | URDF / Xacro |

---

## 3. System Architecture

```text
                    Saved Occupancy Map
                            |
                            v
2D LiDAR ---------> ICP Localization
                            |
                            v
                        ICP Pose
                            |
                            |
Wheel Odometry ------------+
                            |
IMU ------------------------+
                            |
                            v
                           EKF
                            |
                            v
                  Estimated Robot Pose
                     [x, y, yaw]
```

The system will eventually provide the TF relationship:

```text
map
 |
 v
odom
 |
 v
base_link
 |
 +---- laser_link
 |
 +---- imu_link
 |
 +---- left_wheel_link
 |
 +---- right_wheel_link
```

---

## 4. Robot Model

A differential-drive AMR model was created using URDF/Xacro.

The robot currently consists of:

- `base_footprint`
- `base_link`
- `left_wheel_link`
- `right_wheel_link`
- `front_caster_link`
- `rear_caster_link`
- `laser_link`
- `imu_link`

### Robot Dimensions

| Parameter | Value |
|---|---:|
| Body Length | 0.60 m |
| Body Width | 0.40 m |
| Body Height | 0.20 m |
| Drive Wheel Radius | 0.08 m |
| Drive Wheel Width | 0.04 m |
| Caster Radius | 0.04 m |

Two passive spherical caster contacts are used to stabilize the differential-drive robot in simulation.

The drive wheels use relatively high friction for traction, while the passive caster contacts use low friction to reduce resistance during rotation.

---

## 5. Current Project Structure

```text
ros2_icp_ekf_localization/
│
├── src/
│   │
│   ├── robot_description/
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── urdf/
│   │       └── amr.urdf.xacro
│   │
│   └── robot_simulation/
│       ├── config/
│       ├── launch/
│       │   └── simulation.launch.py
│       └── worlds/
│           └── empty_world.sdf
│
├── README.md
└── .gitignore
```

Additional packages will be added as localization development progresses.

Planned structure:

```text
src/
├── robot_description/
├── robot_simulation/
├── icp_localization/
├── ekf_localization/
└── localization_bringup/

maps/
docs/
results/
```

---

# 6. Development Timeline

## Phase 1 — Development Environment

**Status: Completed**

- Installed Ubuntu 22.04
- Installed ROS2 Humble
- Configured ROS2 workspace
- Configured colcon build environment
- Installed RViz2
- Installed Gazebo Fortress / ROS-Gazebo integration

---

## Phase 2 — AMR URDF/Xacro Model

**Status: Completed**

Created a differential-drive AMR using Xacro.

Implemented:

- `base_footprint`
- `base_link`
- Left drive wheel
- Right drive wheel
- LiDAR frame
- IMU frame
- Front passive caster
- Rear passive caster

Defined:

- Visual geometry
- Collision geometry
- Mass
- Inertia
- Joint relationships
- Sensor mounting positions

---

## Phase 3 — TF and Robot Model Validation

**Status: Completed**

Validated the robot model using:

```bash
xacro
check_urdf
robot_state_publisher
joint_state_publisher
RViz2
```

Verified TF hierarchy and robot geometry in RViz2.

Example:

```text
base_footprint
      |
      v
   base_link
      |
      +---- left_wheel_link
      |
      +---- right_wheel_link
      |
      +---- laser_link
      |
      +---- imu_link
      |
      +---- front_caster_link
      |
      +---- rear_caster_link
```

---

## Phase 4 — Gazebo Simulation Environment

**Status: In Progress**

Created the first Gazebo world:

```text
empty_world.sdf
```

Current simulation contains:

- Ground plane
- Directional light
- Physics system
- AMR model

The URDF/Xacro robot has successfully been spawned into Gazebo.

Passive caster contacts were added because the original two-wheel model could rotate around the drive-wheel axis under gravity.

### Current Milestone

```text
URDF/Xacro
    ↓
robot_state_publisher
    ↓
Gazebo Fortress
    ↓
Robot Spawn
    ↓
Stable Ground Contact
```

---

## Phase 5 — Differential Drive

**Status: Planned**

Next objectives:

- Configure differential-drive system
- Connect left/right wheel joints
- Receive `/cmd_vel`
- Implement forward/backward motion
- Implement rotation
- Generate wheel odometry

Expected topics:

```text
/cmd_vel
/odom
```

Expected TF:

```text
odom → base_link
```

---

## Phase 6 — LiDAR Simulation

**Status: Planned**

Add a simulated 2D LiDAR sensor.

Target output:

```text
/scan
```

Tasks:

- Configure LiDAR range
- Configure angular resolution
- Configure update rate
- Connect Gazebo LaserScan to ROS2
- Visualize LaserScan in RViz2

---

## Phase 7 — IMU Simulation

**Status: Planned**

Add simulated IMU measurements.

Target output:

```text
/imu
```

Measurements will include:

- Angular velocity
- Linear acceleration
- Orientation where applicable

---

## Phase 8 — Mapping

**Status: Planned**

Create a warehouse-like simulation environment containing:

- Walls
- Corridors
- Static obstacles
- Distinct geometric features

Mapping pipeline:

```text
Gazebo World
      |
      v
2D LiDAR /scan
      |
      v
SLAM Toolbox
      |
      v
Occupancy Grid
```

The resulting map will be stored as:

```text
maps/
├── warehouse_map.pgm
└── warehouse_map.yaml
```

---

## Phase 9 — ICP Localization

**Status: Planned**

Implement 2D ICP localization in C++.

Main processing pipeline:

```text
/map
  |
  v
Reference Point Cloud
        +
Current LiDAR Scan
        |
        v
Correspondence Search
        |
        v
Rigid Transform Estimation
        |
        v
Iterative Optimization
        |
        v
ICP Pose
[x, y, yaw]
```

Planned implementation topics include:

- LaserScan-to-point-cloud conversion
- Nearest-neighbor correspondence search
- Outlier rejection
- SVD-based rigid transformation estimation
- Iterative pose optimization
- Convergence criteria
- ICP fitness/error evaluation

---

## Phase 10 — EKF Sensor Fusion

**Status: Planned**

Implement an Extended Kalman Filter in C++.

Inputs:

```text
Wheel Odometry
IMU
ICP Pose
```

Output:

```text
Fused Localization Pose
```

The EKF will contain:

1. State definition
2. Motion model
3. Prediction step
4. Covariance propagation
5. Measurement model
6. Kalman gain calculation
7. Measurement update
8. Angle normalization

---

## Phase 11 — Localization Evaluation

**Status: Planned**

Compare the estimated localization result against Gazebo ground truth.

Metrics:

- X position error
- Y position error
- Yaw error
- Translation RMSE
- ICP convergence rate
- Localization trajectory drift

Comparison:

```text
Ground Truth
     |
     +----------------------+
                            |
Wheel Odometry ------------|---- Error Analysis
                            |
ICP -----------------------|
                            |
ICP + EKF -----------------+
```

---

# 7. Final Goal

The final system will demonstrate:

```text
Gazebo AMR
    |
    +---- 2D LiDAR
    |
    +---- Wheel Odometry
    |
    +---- IMU
           |
           v
    ICP Localization
           |
           v
      EKF Fusion
           |
           v
    Estimated Pose
           |
           v
 Ground Truth Evaluation
```

The project focuses on understanding and implementing the core algorithms behind AMR localization, including coordinate transforms, scan matching, state estimation, sensor fusion, and quantitative localization evaluation.

---

# 8. Build

```bash
cd ~/ros2_icp_ekf_localization

colcon build --symlink-install

source install/setup.bash
```

---

# 9. Run Robot Model in RViz2

```bash
ros2 launch robot_description display.launch.py
```

---

# 10. Run Gazebo Simulation

```bash
ros2 launch robot_simulation simulation.launch.py
```

---

## Progress

```text
[████████░░░░░░░░░░░░] AMR / Simulation
[░░░░░░░░░░░░░░░░░░░░] ICP Localization
[░░░░░░░░░░░░░░░░░░░░] EKF Sensor Fusion
[░░░░░░░░░░░░░░░░░░░░] Evaluation
```

Current milestone:

**AMR URDF/Xacro model and basic Gazebo simulation completed.**