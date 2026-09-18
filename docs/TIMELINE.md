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

### Custom ICP Localization — Input Data Preparation
- Created the `icp_localization` C++ package
- Added ROS2 `/scan` subscription using `sensor_msgs/msg/LaserScan`
- Filtered invalid LiDAR range values
- Converted each valid LiDAR beam from polar coordinates to `(x, y)` points
- Published the converted scan as `sensor_msgs/msg/PointCloud2` on `/icp_scan_points`
- Verified the raw `/scan` data and `/icp_scan_points` overlap correctly in RViz2
- Added ROS2 `/map` subscription using `nav_msgs/msg/OccupancyGrid`
- Matched the Map Server QoS using `Reliable + Transient Local`
- Extracted occupied map cells using an occupancy threshold of `65`
- Converted the 1D OccupancyGrid index into row / column coordinates
- Converted occupied grid-cell centers into metric map-frame `(x, y)` coordinates
- Published the reference map points as `sensor_msgs/msg/PointCloud2` on `/icp_map_points`
- Configured `/icp_map_points` with `Reliable + Transient Local` QoS
- Verified the Occupancy Grid and reference point cloud overlap correctly in RViz2

### ICP Input Validation Artifacts
- Added `docs/images/10_laserscan_pointcloud_overlap_rviz.gif`
- Added `docs/images/11_occupancygrid_reference_pointcloud_rviz.gif`

### Issues Resolved
- Identified that a one-time `/icp_map_points` publication could be missed by RViz2 when using the default volatile QoS
- Changed the reference point-cloud publisher to `Reliable + Transient Local` so late subscribers can receive the latest map point cloud
- Used the same durable QoS profile for `/map` subscription to match Nav2 Map Server
- Confirmed that temporary `map -> odom` static TF may be used only for RViz2 input visualization while the real localization transform is not yet implemented

### Current Milestone
- IMU Simulation: Completed
- Benchmark Localization World: Completed
- World / Initial Pose Launch Arguments: Completed
- Odometry TF Integration: Completed
- SLAM Toolbox Mapping: Completed
- Occupancy Grid Generation: Completed
- Map Save: Completed
- Saved Map Reload Validation: Completed
- ICP C++ Package: In Progress
- LaserScan -> PointCloud2: Completed
- OccupancyGrid -> Reference PointCloud: Completed
- ICP Input RViz2 Validation: Completed

### Next Step
- Transform `/icp_scan_points` from `laser_link` into the `map` coordinate frame
- Use odometry / TF as the initial pose estimate
- Implement nearest-neighbor correspondence search
- Add correspondence filtering / outlier rejection
- Implement 2D rigid transform estimation and ICP iteration

## 2026-09-18

### ICP Map Reference Persistence
- Changed the map reference point vector from a `mapCallback()` local variable to the class member `map_points_`
- Preserved the extracted OccupancyGrid reference points after `mapCallback()` returns
- Verified that `scanCallback()` can access the saved reference points for later ICP matching

### LaserScan TF Transformation
- Added TF2 dependencies to the custom ICP package
- Looked up the `laser_link -> odom` transform through the ROS2 TF tree
- Extracted translation and yaw from the TF transform
- Applied the 2D rotation / translation equation to each LiDAR point
- Published odometry-frame scan points on `/icp_scan_points_odom`
- Verified the transformed scan dynamically in RViz2 while the robot moves

### Map-Frame Scan Preparation
- Added a `Pose2D` structure for the current `map -> odom` estimate
- Initialized the benchmark transform estimate to `x=0.0`, `y=0.0`, `yaw=0.0` after validating the identity alignment condition
- Transformed `odom_points` into `map_scan_points`
- Published the current map-frame LiDAR observation on `/icp_scan_points_map`
- Confirmed that `/icp_scan_points_map` is an ICP Source point cloud, not a newly generated map

### Map-Related QoS Cleanup
- Removed duplicated map QoS definitions
- Added one shared `map_related_qos` profile
- Applied `KeepLast(1) + Reliable + Transient Local` to the `/map` subscriber, `/icp_map_points`, and `/icp_scan_points_map`
- Kept the intermediate `/icp_scan_points` and `/icp_scan_points_odom` streams on their existing continuous-stream QoS settings

### ICP Source / Target RViz2 Validation
- Visualized `/icp_map_points` as the saved-map Target point cloud
- Visualized `/icp_scan_points_map` as the current LiDAR Source point cloud
- Used different FlatColor settings in RViz2 to make the two point sets easy to compare
- Verified that the current scan follows the same wall geometry as the saved map
- Observed a small residual map/scan offset during rotation, which is acceptable at the current pre-ICP stage

### Current Limitation / Observation
- The current TF lookup uses `tf2::TimePointZero`, which returns the latest available transform
- During robot rotation, LiDAR measurement time and TF time can differ slightly and produce a visible residual offset
- Timestamp-aligned TF lookup is planned before quantitative localization evaluation
- The temporary identity `map -> odom` static TF remains a visualization-only aid and will be removed when custom localization publishes the real transform

### ICP Input Validation Artifacts
- Added `docs/images/12_laserscan_odom_transform_rviz.gif`
- Added `docs/images/13_icp_map_scan_alignment_rviz.png`

### Current Milestone
- ICP C++ Package: In Progress
- LaserScan -> 2D PointCloud: Completed
- OccupancyGrid -> Reference PointCloud: Completed
- Persistent `map_points_`: Completed
- `laser_link -> odom` Scan Transformation: Completed
- `odom -> map` Scan Transformation: Completed
- `/icp_scan_points_map`: Completed
- Map-Related QoS Cleanup: Completed
- Map Target / Scan Source RViz2 Validation: Completed
- Nearest-Neighbor Correspondence Search: Next

### Next Step
- Implement brute-force nearest-neighbor correspondence search
- Compute squared Euclidean distance between each Source scan point and Target map point
- Validate correspondence count and distance distribution
- Add maximum correspondence-distance filtering / outlier rejection
- Continue to 2D rigid-transform estimation after correspondence validation

