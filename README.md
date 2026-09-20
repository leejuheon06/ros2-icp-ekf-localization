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
│   ├── robot_simulation/
│   │   ├── config/
│   │   │   ├── bridge.yaml
│   │   │   └── slam_toolbox.yaml
│   │   ├── launch/
│   │   │   ├── simulation.launch.py
│   │   │   └── localization_icp.launch.py
│   │   └── worlds/
│   │       ├── empty_world.sdf
│   │       └── localization_world.sdf
│   │
│   └── icp_localization/
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── src/
│           └── icp_localization_node.cpp
│
├── maps/
│   ├── localization_map.pgm
│   └── localization_map.yaml
│
├── docs/
│   ├── COMMANDS.md
│   ├── TIMELINE.md
│   └── images/
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

### RViz2 Robot Model Validation

![RViz2 Robot Model](docs/images/01_robot_model.png)

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

### Differential Drive Validation

Forward motion:

![Differential Drive Straight](docs/images/02_differential_drive_gazebo_straight.gif)

Rotation test:

![Differential Drive Turn](docs/images/02_differential_drive_gazebo_turn.gif)

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

### ROS2 Bridge Validation

![ROS2 Gazebo Bridge](docs/images/03_differential_drive_ROS2_bridge.gif)

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

### Joint State and LiDAR Visualization

![Joint States and LiDAR Scan](docs/images/04_joint_states_lidar_scan_rviz.gif)

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

### Benchmark World Validation

![Localization Benchmark World](docs/images/07_localization_world_gazebo.png.png)

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

### SLAM Toolbox Mapping Validation

![SLAM Toolbox Mapping](docs/images/08_slam_toolbox_mapping_rviz.gif)

Final generated Occupancy Grid:

![SLAM Toolbox Generated Map](docs/images/08_slam_toolbox_generated_map_rviz.png)

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

![Saved Map Reload Validation](docs/images/09_saved_map_reload_rviz.png)

When only Map Server is running, `map -> odom` is not published. Therefore, RViz2 may report that the `map` Fixed Frame does not exist in the TF tree even though the saved Occupancy Grid itself is loaded correctly. A localization node will provide the required map-relative transform in later stages.

---

## Phase 9 — ICP Localization

**Status: In Progress**

The custom 2D ICP localization package is implemented in C++ using the saved Occupancy Grid, current 2D LiDAR scan, odometry-frame TF, and an internally maintained `map -> odom` estimate.

Current processing pipeline:

```text
Saved Map /map
      |
      v
OccupancyGrid -> Occupied Cells
      |
      v
map_points_  [Target / map frame]
      |
      +-----------------------------------------------------------+
                                                                  |
Current LiDAR /scan                                               |
      |                                                           |
      v                                                           |
LaserScan -> 2D Points [laser_link]                               |
      |                                                           |
      v                                                           |
Timestamp-aligned TF: laser_link -> odom                          |
      |                                                           |
      v                                                           |
odom_points                                                       |
      |                                                           |
      v                                                           |
Current T_map_odom                                                |
      |                                                           |
      v                                                           |
map_scan_points [Source / map frame] -----------------------------+
                                                                  |
                                                                  v
                                                Nearest Neighbor Search
                                                                  |
                                                                  v
                                                     Correspondence Pairs
                                                                  |
                                                                  v
                                                        Outlier Rejection
                                                                  |
                                                                  v
                                                 Source / Target Centroids
                                                                  |
                                                                  v
                                             2D Rigid Correction Estimation
                                                delta_x / delta_y / delta_yaw
                                                                  |
                                                                  v
                                                   T_map_odom Update
                                                                  |
                                                                  v
                                              Repeat Until Convergence
```

### ICP ROS2 Package

Created the C++ package:

```text
src/icp_localization/
├── CMakeLists.txt
├── package.xml
└── src/
    └── icp_localization_node.cpp
```

Current dependencies:

- `rclcpp`
- `sensor_msgs`
- `nav_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`

### LaserScan to 2D Point Cloud

Implemented:

- ROS2 `/scan` subscription
- Invalid range filtering using finite values and sensor range limits
- LiDAR beam-angle calculation using `angle_min` and `angle_increment`
- Polar-to-Cartesian conversion
- Internal `Point2D` representation
- ROS2 `sensor_msgs/msg/PointCloud2` publication on `/icp_scan_points`

Coordinate conversion:

```text
angle = angle_min + i * angle_increment
x = range * cos(angle)
y = range * sin(angle)
```

The original `/scan` visualization and the converted `/icp_scan_points` were displayed simultaneously in RViz2. The point sets overlapped correctly, validating the scan conversion used as the first ICP input-preparation step.

![LaserScan PointCloud Overlap](docs/images/10_laserscan_pointcloud_overlap_rviz.gif)

### OccupancyGrid to Reference Point Cloud

Implemented:

- ROS2 `/map` subscription
- Occupied-cell extraction from `nav_msgs/msg/OccupancyGrid`
- 1D map index conversion to row and column
- Grid-cell center conversion to map-frame metric coordinates
- Persistent `map_points_` class member so the reference points remain available after `mapCallback()` returns
- ROS2 `sensor_msgs/msg/PointCloud2` publication on `/icp_map_points`

Grid conversion:

```text
column = index % width
row    = index / width

x = origin_x + (column + 0.5) * resolution
y = origin_y + (row + 0.5) * resolution
```

Occupied cells currently use an occupancy threshold of `65` or greater. The generated reference points follow the walls and obstacles of the saved Occupancy Grid in RViz2.

![OccupancyGrid Reference PointCloud](docs/images/11_occupancygrid_reference_pointcloud_rviz.gif)

### LaserScan to Odom Frame Transformation

The current LiDAR points are transformed from `laser_link` into the odometry coordinate frame using the ROS2 TF tree.

```text
laser_link points
      |
      v
TF lookup: odom <- laser_link
      |
      v
2D rotation + translation
      |
      v
odom_points
```

The transform translation and yaw are applied manually using:

```text
x' = cos(yaw) * x - sin(yaw) * y + tx
y' = sin(yaw) * x + cos(yaw) * y + ty
```

The transformed points are published on:

```text
Topic: /icp_scan_points_odom
Frame: odom
```

Dynamic RViz2 validation confirmed that the transformed scan follows the robot motion in the odometry frame.

![LaserScan Odom Transformation](docs/images/12_laserscan_odom_transform_rviz.gif)

### Odom to Map Frame Transformation

For ICP comparison, both the current scan and saved-map reference points use the `map` coordinate frame.

A `Pose2D` structure stores the current internal `map -> odom` estimate:

```text
[x, y, yaw]
```

The benchmark starts from an identity initial estimate:

```text
x   = 0.0
y   = 0.0
yaw = 0.0
```

The odometry-frame scan points are transformed using this estimate and published as:

```text
Topic: /icp_scan_points_map
Frame: map
```

This topic does **not** create a new map. It represents the current LiDAR observation expressed in the saved map coordinate frame.

### Nearest-Neighbor Correspondence Search

Each current Source scan point is matched with the closest Target map point.

The first implementation intentionally uses brute-force search:

```text
For each Source scan point
      |
      v
Compare against every Target map point
      |
      v
Select minimum squared Euclidean distance
      |
      v
Create Source <-> Target correspondence
```

Squared distance is used during nearest-neighbor comparison:

```text
distance^2 = dx^2 + dy^2
```

This avoids an unnecessary square root for every map-point comparison.

The brute-force implementation is retained at this stage because it makes the ICP matching process explicit and easy to validate. A KD-tree can be introduced later for performance optimization.

### Correspondence Filtering / Outlier Rejection

A nearest map point is not automatically a trustworthy correspondence. Distant pairs can be caused by measurement noise, dynamic objects, map mismatch, incorrect nearest-neighbor association, or initial pose error.

A maximum correspondence-distance threshold is therefore applied before rigid-transform estimation.

The normal development value is currently:

```text
Maximum correspondence distance: 0.15 m
```

For validation, the threshold was temporarily reduced to `0.05 m`. During this test, the system correctly rejected distant correspondences while preserving the identity:

```text
Raw = Valid + Rejected
```

Observed validation examples included:

```text
Raw: 360 | Valid: 358 | Rejected: 2
Raw: 360 | Valid: 355 | Rejected: 5
Raw: 360 | Valid: 351 | Rejected: 9
```

The threshold was then restored to `0.15 m` so that the initial ICP alignment has more tolerance before convergence.

### Centroid and 2D Rigid Correction

Only valid correspondences are used to estimate the rigid correction.

The Source and Target centroids are calculated first:

```text
source_centroid = mean(Source points)
target_centroid = mean(Target points)
```

Subtracting each centroid places both point sets around a common origin so that the rotation component can be estimated independently of the overall translation.

The 2D incremental rotation is calculated from the centered correspondence pairs, and translation is then recovered from:

```text
t = target_centroid - R * source_centroid
```

The resulting incremental correction is:

```text
delta_x
delta_y
delta_yaw
```

The correction is composed with the previous pose estimate as:

```text
T_new = Delta_T * T_old
```

rather than simply adding translation values without accounting for rotation.

### ICP Iteration and Convergence

The ICP solver now repeats correspondence search and rigid correction multiple times for the **same LaserScan frame**.

```text
One LaserScan
      |
      v
Correspondence Search
      |
      v
Outlier Rejection
      |
      v
Rigid Correction
      |
      v
Update Temporary Pose
      |
      +----> Repeat
```

Current convergence settings:

```text
Maximum iterations: 10
Translation convergence threshold: 0.001 m
Rotation convergence threshold: 0.001 rad
```

The iteration stops early when both the translation correction magnitude and rotation correction are sufficiently small.

Stationary validation confirmed convergence. After the initial correction, subsequent scans converged at the first iteration with correction values effectively equal to zero.

Example observed result:

```text
ICP Iteration | iter: 1/10 | converged: true
Raw: 360 | Valid: 360 | Rejected: 0
Raw mean: 0.01372 m
mean: 0.01372 -> 0.01372 m

Final Correction
dx: 0.000000 m
dy: 0.000000 m
dyaw: 0.000000 rad
map_to_odom: (0.01555, 0.02279, -0.00001)
```

An earlier correction also reduced the correspondence mean distance from approximately `0.02343 m` to `0.01751 m`, confirming that the rigid correction moved the Source point set toward the Target point set before convergence.

### LaserScan Timestamp-Aligned TF

The previous implementation used:

```text
tf2::TimePointZero
```

which requested the latest available TF.

This can introduce a temporal mismatch while the robot is moving because the LiDAR measurement timestamp and latest TF timestamp may differ.

The TF lookup now uses the actual LaserScan timestamp:

```text
scan_time = msg->header.stamp
lookupTransform(
    target = odom,
    source = laser frame,
    time   = scan_time
)
```

A `0.1 s` lookup timeout is used.

This change ensures that a LaserScan is transformed using the robot pose corresponding to the scan measurement time rather than an unrelated latest transform.

### Simulation Clock Bridge

The localization nodes use:

```text
use_sim_time = true
```

During integration testing, `/clock` existed in the ROS2 graph but initially had:

```text
Publisher count: 0
```

As a result, ROS simulation time did not advance and time-dependent throttled logs / timestamp processing did not behave as expected.

Gazebo simulation time was added to the existing ROS-Gazebo bridge configuration:

```text
Gazebo /clock
      |
      v
ros_gz_bridge
      |
      v
ROS2 /clock
```

The `/clock` bridge is managed together with the other bridge topics in:

```text
src/robot_simulation/config/bridge.yaml
```

This allows the Map Server and ICP node to keep `use_sim_time=true` without requiring a separate Clock bridge terminal.

### Integrated ICP Localization Launch

A dedicated launch file was added:

```text
src/robot_simulation/launch/localization_icp.launch.py
```

It starts the current localization test environment from a single command:

```text
Gazebo benchmark simulation
robot_state_publisher
robot spawn
ROS-Gazebo bridge
Nav2 Map Server
Map Server lifecycle configure / activate
temporary identity map -> odom TF
icp_localization_node
```

Run:

```bash
ros2 launch robot_simulation localization_icp.launch.py
```

The dedicated launch keeps the current ICP development environment repeatable while reducing the previous multi-terminal startup sequence.

### Map-Frame ICP Visualization

The two ICP point sets remain directly visualizable in RViz2:

```text
Target
/icp_map_points
Frame: map
Source: saved OccupancyGrid

Source
/icp_scan_points_map
Frame: map
Source: current LiDAR observation
```

The map-related ROS2 interfaces currently share one development / RViz2 QoS profile:

```text
History: Keep Last
Depth: 1
Reliability: Reliable
Durability: Transient Local
```

The shared profile is used by:

- `/map` subscriber
- `/icp_map_points` publisher
- `/icp_scan_points_map` publisher

![ICP Map-Frame Input Alignment](docs/images/13_icp_map_scan_alignment_rviz.png)

### Current Limitation

The calculated `map_to_odom_` correction is currently maintained and applied **inside the custom ICP node**.

The real ROS2 dynamic TF:

```text
map -> odom
```

is not yet broadcast by the ICP node.

Therefore, `localization_icp.launch.py` still starts a temporary identity `map -> odom` static transform to keep the ROS TF tree connected for RViz2 and current development validation.

This temporary static transform must be removed when the custom ICP node begins broadcasting its calculated dynamic `map -> odom` transform.

### Current ICP Status

```text
[✓] LaserScan -> 2D Point conversion
[✓] OccupancyGrid -> persistent map reference points
[✓] laser_link -> odom transformation
[✓] odom -> map transformation
[✓] Map Target / Scan Source RViz2 validation
[✓] Brute-force nearest-neighbor correspondence search
[✓] Correspondence distance statistics
[✓] Maximum-distance outlier rejection
[✓] Source / Target centroid calculation
[✓] 2D delta_x / delta_y / delta_yaw estimation
[✓] Internal map_to_odom_ correction update
[✓] ICP iteration for one LaserScan
[✓] Convergence condition
[✓] Gazebo /clock -> ROS2 /clock bridge
[✓] LaserScan timestamp-aligned TF lookup
[✓] Single-command localization development launch

[ ] Dynamic map -> odom TF broadcast from custom ICP
[ ] Remove temporary static map -> odom publisher
[ ] Moving-robot localization validation
[ ] Quantitative ICP accuracy / runtime evaluation
```

Next implementation steps:

- Broadcast the calculated `map -> odom` transform from the custom ICP node
- Remove the temporary static identity `map -> odom` publisher
- Validate ICP convergence while the robot translates and rotates
- Measure localization accuracy and processing time
- Continue toward EKF integration

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
[██████████████░░░░░░] ICP Localization
[░░░░░░░░░░░░░░░░░░░░] EKF Sensor Fusion
[██░░░░░░░░░░░░░░░░░░] Evaluation Infrastructure
```

Current milestone:

**AMR simulation, sensor integration, benchmark mapping, and saved-map validation are complete. The custom ICP pipeline now includes nearest-neighbor correspondence search, outlier rejection, centroid-based 2D rigid correction, iterative convergence, an internal `map_to_odom_` update, Gazebo `/clock` bridging, and LaserScan timestamp-aligned TF lookup. Next: broadcast the calculated dynamic `map -> odom` TF and remove the temporary static transform.**
