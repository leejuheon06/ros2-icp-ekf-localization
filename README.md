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
│       │   ├── bridge.yaml
│       │   └── slam_toolbox.yaml
│       ├── launch/
│       │   └── simulation.launch.py
│       └── worlds/
│           ├── empty_world.sdf
│           └── localization_world.sdf
│
├── maps/
│   ├── localization_map.pgm
│   └── localization_map.yaml
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

**Status: Completed**

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

**Status: Completed**

Implemented:

- Gazebo Fortress Differential Drive system
- `left_wheel_joint` / `right_wheel_joint` connection
- Wheel separation: `0.44 m`
- Wheel radius: `0.08 m`
- `/cmd_vel` velocity command
- Wheel odometry generation

Verified motion:

- `linear.x > 0` → Forward (+X / LiDAR direction)
- `linear.x < 0` → Backward
- `angular.z > 0` → Counter-clockwise rotation
- `angular.z < 0` → Clockwise rotation

During the initial motion test, positive `linear.x` moved the robot backward and positive `angular.z` rotated the robot clockwise.

The issue was caused by the wheel joint axis direction.

Changed both wheel joint axes from:

```xml
<axis xyz="0 0 1"/>
```

to:

```xml
<axis xyz="0 0 -1"/>
```

After rebuilding and restarting Gazebo, forward/backward motion and left/right rotation were verified successfully.

Current Gazebo topics:

```text
/cmd_vel
/model/icp_ekf_amr/odometry
/model/icp_ekf_amr/tf
```

Next objective:

```text
ROS2 /cmd_vel
      |
      v
ros_gz_bridge
      |
      v
Gazebo Differential Drive
      |
      v
ROS2 /odom
```

---

## Phase 5-1 — ROS2 ↔ Gazebo Bridge

**Status: Completed**

Integrated `ros_gz_bridge` into `simulation.launch.py` so that the ROS2 and Gazebo communication bridge starts automatically with the simulation.

Implemented:

- ROS2 `/cmd_vel` → Gazebo Differential Drive
- Gazebo odometry → ROS2 `/odom`
- Automatic bridge startup from `simulation.launch.py`
- Verified ROS2 velocity command without using `ign topic`
- Verified ROS2 odometry output

Target communication flow:

```text
ROS2 /cmd_vel
      |
      v
ros_gz_bridge
      |
      v
Gazebo Differential Drive
      |
      v
AMR Motion
      |
      v
Gazebo Odometry
      |
      v
ros_gz_bridge
      |
      v
ROS2 /odom
```

Validation:

```text
ros2 launch robot_simulation simulation.launch.py
        ↓
parameter_bridge automatically started
        ↓
ROS2 /cmd_vel available
        ↓
AMR moves correctly
        ↓
ROS2 /odom available
        ↓
Odometry values change during motion
```

With this step complete, the Differential Drive simulation can now be controlled entirely through ROS2 topics.

---

## Phase 5-2 — Gazebo Joint State Integration

**Status: Completed**

Integrated Gazebo wheel joint states with ROS2 so that the continuous drive-wheel joints can be represented correctly in the ROS2 TF tree and RViz2.

Implemented:

- Gazebo `JointStatePublisher` for `left_wheel_joint` and `right_wheel_joint`
- Gazebo joint state → ROS2 `/joint_states` bridge
- `robot_state_publisher` integration using wheel joint states
- Dynamic TF generation for `left_wheel_link` and `right_wheel_link`
- RViz2 wheel visualization without relying on a standalone `joint_state_publisher` for simulation state

Data flow:

```text
Gazebo Wheel Joints
      |
      v
Gazebo JointStatePublisher
      |
      v
ros_gz_bridge
      |
      v
ROS2 /joint_states
      |
      v
robot_state_publisher
      |
      v
/tf
      |
      v
RViz2
```

This resolved the issue where fixed links were visible in RViz2 but the continuous left/right wheel links were missing because wheel joint states were unavailable to `robot_state_publisher`.

---

## Phase 6 — LiDAR Simulation

**Status: Completed**

Integrated a simulated 360° 2D GPU LiDAR sensor and connected the Gazebo LaserScan output to ROS2.

Configuration:

- Horizontal field of view: `360°`
- Samples: `360`
- Minimum range: `0.10 m`
- Maximum range: `10.0 m`
- Update rate: `10 Hz`
- ROS2 output: `/scan`

Implemented:

- Gazebo GPU LiDAR sensor on `laser_link`
- Gazebo Sensors system using Ogre2
- Gazebo `/scan` → ROS2 `/scan` bridge
- LiDAR sensor frame alignment with the ROS2 TF tree
- RViz2 LaserScan visualization
- Static obstacle detection test in Gazebo

Data flow:

```text
Gazebo GPU LiDAR
      |
      v
Gazebo /scan
      |
      v
ros_gz_bridge
      |
      v
ROS2 /scan
      |
      v
RViz2 LaserScan
```

During initial RViz2 validation, LaserScan messages were dropped because the Gazebo LiDAR frame did not match an available ROS2 TF frame. Aligning the sensor frame with `laser_link` resolved the RViz Message Filter queue overflow.

Final validation confirmed that static obstacles in the Gazebo world are detected and displayed as LaserScan points in RViz2.

---

## Phase 7 — IMU Simulation

**Status: Completed**

Integrated a simulated IMU sensor into the AMR model and bridged the Gazebo IMU output to ROS2.

Configuration:

- Sensor link: `imu_link`
- ROS2 output: `/imu`
- Message type: `sensor_msgs/msg/Imu`
- Update rate: `50 Hz`
- Sensor frame: `imu_link`

Implemented:

- Gazebo IMU system plugin
- IMU sensor attached to `imu_link`
- Gazebo `/imu` → ROS2 `/imu` bridge
- IMU frame alignment with the ROS2 TF tree
- Orientation, angular velocity, and linear acceleration validation
- Robot rotation test using `angular_velocity.z`

Data flow:

```text
Gazebo IMU
      |
      v
Gazebo /imu
      |
      v
ros_gz_bridge
      |
      v
ROS2 /imu
      |
      v
Localization Pipeline
```

During initial validation, the IMU message used a Gazebo-scoped sensor frame instead of `imu_link`. The sensor frame was aligned with the ROS2 TF tree so that `/imu` can be used consistently in the later EKF stage.

---

## Phase 8 — Evaluation Environment and Mapping

**Status: Completed**

Created a fixed benchmark environment for mapping and future localization evaluation.

The evaluation world is stored as:

```text
src/robot_simulation/worlds/localization_world.sdf
```

The environment intentionally contains asymmetric geometric features, including:

- Outer walls
- Different-sized box landmarks
- A rotated rectangular obstacle
- A central rectangular structure
- An L-shaped wall feature

The asymmetric layout reduces geometric ambiguity and provides distinct LiDAR features for scan matching and localization evaluation.

### World and Initial Pose Selection

`simulation.launch.py` was extended with launch arguments so that the simulation world and robot initial pose can be selected without modifying the launch file.

Example:

```bash
ros2 launch robot_simulation simulation.launch.py \
world:=localization_world.sdf \
x:=0.0 \
y:=-3.8 \
yaw:=1.5708
```

This allows the benchmark environment and initial pose to remain fixed across future localization experiments.

### Odometry TF Integration

The Gazebo Differential Drive configuration was extended to publish the odometry TF relationship:

```text
odom
  |
  v
base_footprint
  |
  v
base_link
```

The Gazebo TF output is bridged to ROS2 `/tf`, allowing SLAM Toolbox and future localization nodes to use the same TF chain.

### SLAM Toolbox Mapping

SLAM Toolbox was configured using:

```text
scan_topic: /scan
odom_frame: odom
map_frame: map
base_frame: base_footprint
mode: mapping
```

The LiDAR range configuration was matched to the simulated sensor:

```text
Minimum range: 0.10 m
Maximum range: 10.0 m
```

Mapping pipeline:

```text
Gazebo localization_world
      |
      v
2D LiDAR /scan
      |
      +------------------+
      |                  |
      v                  v
LaserScan        odom -> base_footprint
      |                  |
      +--------+---------+
               |
               v
         SLAM Toolbox
               |
         +-----+-----+
         |           |
         v           v
       /map      map -> odom
```

A 2D Occupancy Grid was successfully generated in RViz2.

Current generated map information:

- Resolution: `0.05 m/cell`
- Width: `199 cells`
- Height: `198 cells`
- ROS2 type: `nav_msgs/msg/OccupancyGrid`

The generated map was saved as:

```text
maps/
├── localization_map.pgm
└── localization_map.yaml
```

### Saved Map Reload Validation

The saved map was reloaded using Nav2 Map Server and validated in RViz2.

The `/map` topic uses a durable map-style QoS profile. During validation, explicitly matching the subscriber QoS was required:

```text
Reliability: Reliable
Durability: Transient Local
```

The saved Occupancy Grid was successfully received and displayed in RViz2 with `Map -> Status: Ok`.

When only Map Server is running, `map -> odom` is not published. Therefore, RViz2 may report that the `map` Fixed Frame does not exist in the TF tree even though the saved Occupancy Grid itself is loaded correctly. A localization node will provide the required map-relative transform in later stages.

Recommended result images:

```text
docs/images/07_localization_world_gazebo.png
docs/images/08_slam_toolbox_generated_map_rviz.png
docs/images/09_saved_map_reload_rviz.png
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

## Phase 11 — Baseline Localization Integration

**Status: Planned**

Prepare baseline localization methods for comparison under the same benchmark conditions.

Comparison methods:

```text
Wheel Odometry Only
Custom ICP
Custom ICP + EKF
Nav2 AMCL
SLAM Toolbox Localization
```

All methods will use the same benchmark world, initial pose, map, and evaluation trajectory where applicable.

---

## Phase 12 — Quantitative Localization Evaluation

**Status: Planned**

Compare each localization result against Gazebo ground truth.

Planned metrics:

- X position error
- Y position error
- Yaw error
- Translation RMSE
- Maximum position error
- Localization trajectory drift
- ICP convergence rate
- Processing time
- Localization update frequency

Comparison:

```text
                    Gazebo Ground Truth
                           |
          +----------------+----------------+
          |                |                |
          v                v                v
      Odometry            ICP          ICP + EKF
          |                |                |
          +----------------+----------------+
                           |
                    Error Analysis
                           |
          +----------------+----------------+
          |                                 |
          v                                 v
       AMCL                      SLAM Toolbox Localization
```

The final evaluation will report quantitative targets, measured results, target achievement status, and comparison graphs.

---

## Phase 13 — Results and Documentation

**Status: Planned**

Planned final result artifacts:

```text
results/
├── trajectory_comparison.png
├── translation_error_time_series.png
├── yaw_error_time_series.png
├── localization_rmse_comparison.png
├── processing_time_comparison.png
└── localization_kpi_summary.png
```

The final README will summarize:

- Quantitative target values
- Actual measured performance
- Target achievement status
- Odometry / ICP / ICP+EKF / AMCL / SLAM Toolbox comparison
- Localization error and processing-time graphs
- Analysis of failure cases and limitations

---

# 7. Final Goal

The final system will demonstrate:

```text
Gazebo Benchmark World
          |
          +---- 2D LiDAR
          |
          +---- Wheel Odometry
          |
          +---- IMU
          |
          v
   Custom ICP Localization
          |
          v
      EKF Fusion
          |
          v
    Estimated Pose
          |
          +-------------------------------+
          |                               |
          v                               v
  Gazebo Ground Truth           Baseline Localization
                                  - Nav2 AMCL
                                  - SLAM Toolbox
          |                               |
          +---------------+---------------+
                          |
                          v
                 Quantitative Evaluation
                 RMSE / Error / Latency
```

The project focuses on understanding and implementing the core algorithms behind AMR localization, including coordinate transforms, scan matching, state estimation, sensor fusion, map-based localization, and quantitative comparison against established ROS2 localization baselines.

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

Run the benchmark localization world with a fixed initial pose:

```bash
ros2 launch robot_simulation simulation.launch.py \
world:=localization_world.sdf \
x:=0.0 \
y:=-3.8 \
yaw:=1.5708
```

---

## Progress

```text
[████████████████████] AMR / Simulation
[████████████████████] Sensors / Mapping
[░░░░░░░░░░░░░░░░░░░░] ICP Localization
[░░░░░░░░░░░░░░░░░░░░] EKF Sensor Fusion
[██░░░░░░░░░░░░░░░░░░] Evaluation Infrastructure
```

Current milestone:

**AMR simulation, ROS2 ↔ Gazebo bridge, wheel joint-state integration, 2D LiDAR, IMU, benchmark localization world, odometry TF, SLAM Toolbox mapping, Occupancy Grid generation, and saved-map reload validation completed. Next: Custom ICP localization.**
