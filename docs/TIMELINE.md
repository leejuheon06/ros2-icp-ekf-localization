# Development Timeline

## 2026-09-04

### AMR URDF Model
- Created differential-drive AMR Xacro model
- Added base_link and base_footprint
- Added left/right drive wheels
- Added LiDAR frame
- Added IMU frame
- Verified RobotModel in RViz2
- Verified TF tree

### Gazebo Simulation
- Created empty_world.sdf
- Added ground plane
- Spawned AMR into Gazebo Fortress
- Identified pitch instability in two-wheel configuration
- Added front/rear passive spherical casters
- Configured low-friction caster contacts

### Issues Resolved
- Fixed unstable two-wheel ground contact

### Next Step
- Configure differential drive
- Test /cmd_vel
- Generate /odom

## 2026-09-05

### Differential Drive
- Added Gazebo Fortress Differential Drive system
- Connected left/right wheel joints
- Configured wheel separation to 0.44 m
- Configured wheel radius to 0.08 m
- Verified `/cmd_vel` input
- Verified Gazebo odometry
- Verified Gazebo TF

### Motion Validation
- Verified forward motion
- Verified backward motion
- Verified counter-clockwise rotation
- Verified clockwise rotation

### Issues Resolved
- Identified reversed forward and rotation direction during initial Differential Drive test
- Changed both wheel joint axes from `<axis xyz="0 0 1"/>` to `<axis xyz="0 0 -1"/>`
- Verified motion follows the ROS coordinate convention

### ROS2 ↔ Gazebo Bridge Integration
- Added `ros_gz_bridge` to `simulation.launch.py`
- Configured the bridge to start automatically with Gazebo simulation
- Connected ROS2 `/cmd_vel` to Gazebo Differential Drive
- Connected Gazebo odometry to ROS2 `/odom`
- Verified bridge operation without manually running `parameter_bridge`

### ROS2 Communication Validation
- Verified `/cmd_vel` is available in ROS2
- Verified ROS2 `/cmd_vel` commands move the AMR correctly
- Verified `/odom` is available in ROS2
- Verified odometry values change while the robot is moving

### Current Milestone
- Differential Drive: Completed
- ROS2 ↔ Gazebo Bridge: Completed
- ROS2 `/cmd_vel`: Completed
- ROS2 `/odom`: Completed

### Next Step
- Add a simulated 2D LiDAR sensor
- Bridge Gazebo LaserScan to ROS2 `/scan`
- Visualize LaserScan data in RViz2

## 2026-09-09

### Gazebo Joint State Integration
- Added Gazebo `JointStatePublisher` for the continuous left/right wheel joints
- Bridged Gazebo wheel joint states to ROS2 `/joint_states`
- Connected `/joint_states` with `robot_state_publisher` for dynamic wheel TF generation
- Verified left/right wheel links are displayed correctly in RViz2 without relying on a standalone `joint_state_publisher` for simulation state

### 2D LiDAR Simulation
- Added a 360° Gazebo GPU LiDAR sensor
- Configured 360 scan samples
- Configured 0.10 m minimum range and 10.0 m maximum range
- Configured 10 Hz update rate
- Added the Gazebo Sensors system using Ogre2
- Bridged Gazebo `/scan` to ROS2 `/scan`
- Visualized `sensor_msgs/msg/LaserScan` in RViz2
- Verified static obstacle detection as LaserScan points in RViz2

### Issues Resolved
- Identified missing wheel visualization in RViz2 as a continuous-joint state / TF issue
- Replaced the temporary standalone joint-state visualization approach with Gazebo-derived wheel joint states
- Identified Gazebo LiDAR sensor-frame mismatch as the cause of repeated RViz2 Message Filter queue overflow
- Aligned the LiDAR sensor frame with the ROS2 TF tree
- Verified that the `discarding message because the queue is full` log no longer occurs during LaserScan visualization

### Current Milestone
- Gazebo Joint State Integration: Completed
- ROS2 `/joint_states`: Completed
- 2D LiDAR Simulation: Completed
- ROS2 `/scan`: Completed
- RViz2 LaserScan Visualization: Completed
- Static Obstacle Detection: Completed

### Next Step
- Add simulated IMU measurements
- Bridge Gazebo IMU data to ROS2 `/imu`
- Validate IMU data before beginning the localization pipeline

## 2026-09-16

### IMU Simulation
- Added the Gazebo IMU system plugin
- Added an IMU sensor to `imu_link`
- Bridged Gazebo `/imu` to ROS2 `/imu`
- Verified `sensor_msgs/msg/Imu`
- Verified approximately 50 Hz IMU output
- Verified orientation, angular velocity, and linear acceleration data
- Verified `angular_velocity.z` changes during robot rotation
- Aligned the IMU message frame with `imu_link`

### Localization Benchmark World
- Created `localization_world.sdf` as a fixed evaluation environment
- Added outer walls and multiple asymmetric geometric landmarks
- Added a rotated obstacle, central rectangular structure, and L-shaped feature
- Chose an asymmetric layout to reduce scan-matching ambiguity and provide distinct LiDAR features
- Preserved `empty_world.sdf` for sensor-level validation

### Simulation Launch Improvements
- Added a `world` launch argument to `simulation.launch.py`
- Added `x`, `y`, `z`, and `yaw` launch arguments for the AMR initial pose
- Verified `empty_world.sdf` and `localization_world.sdf` can be selected from the launch command
- Fixed the benchmark initial pose to `x=0.0`, `y=-3.8`, `yaw=1.5708` for future repeatable evaluation

### Odometry TF Integration
- Configured the Gazebo Differential Drive odometry frame as `odom`
- Configured the child frame as `base_footprint`
- Bridged the Gazebo Differential Drive TF output to ROS2 `/tf`
- Verified `odom -> base_footprint` using `tf2_echo`
- Completed the TF chain required by SLAM Toolbox below the `map` frame

### SLAM Toolbox Mapping
- Installed and configured SLAM Toolbox for online asynchronous mapping
- Configured `scan_topic` as `/scan`
- Configured `odom_frame` as `odom`
- Configured `map_frame` as `map`
- Configured `base_frame` as `base_footprint`
- Matched the SLAM Toolbox laser range to the simulated LiDAR range of `0.10 m` to `10.0 m`
- Verified `/map` publication
- Verified `map -> odom` TF while SLAM Toolbox is running
- Generated a 2D Occupancy Grid of the benchmark environment in RViz2

### Occupancy Map Save
- Saved the generated Occupancy Grid using Nav2 Map Saver
- Created `maps/localization_map.pgm`
- Created `maps/localization_map.yaml`
- Verified the saved map files and metadata
- Generated map resolution: `0.05 m/cell`
- Generated map size: `199 x 198` cells

### Saved Map Reload Validation
- Reloaded `localization_map.yaml` using Nav2 Map Server
- Configured and activated the Map Server Lifecycle Node
- Verified `/map` type as `nav_msgs/msg/OccupancyGrid`
- Verified the saved map is displayed correctly in RViz2
- Verified `Map -> Status: Ok`

### Issues Resolved
- Identified that `/map` could exist while a default subscriber still received no map data because of QoS mismatch
- Matched `/map` subscriber QoS using `Reliable` and `Transient Local`
- Verified OccupancyGrid data after explicitly matching QoS
- Confirmed that Map Server does not publish `map -> odom`; therefore a missing `map` Fixed Frame in the TF tree is expected when no SLAM/localization node is active
- Observed an RViz2 GLSL map-rendering warning and confirmed that it does not block map visualization when the Map display remains `Status: Ok`

### Current Milestone
- IMU Simulation: Completed
- Benchmark Localization World: Completed
- World / Initial Pose Launch Arguments: Completed
- Odometry TF Integration: Completed
- SLAM Toolbox Mapping: Completed
- Occupancy Grid Generation: Completed
- Map Save: Completed
- Saved Map Reload Validation: Completed

### Next Step
- Begin Custom ICP Localization implementation
- Convert the saved occupancy map into a reference point cloud
- Convert ROS2 `/scan` LaserScan data into 2D points
- Implement correspondence search and rigid transform estimation step by step
- Validate ICP pose output before integrating the EKF

