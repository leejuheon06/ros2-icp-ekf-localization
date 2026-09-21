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

## 2026-09-20

### Nearest-Neighbor Correspondence Search
- Added a `Correspondence` structure containing Source point, Target point, and squared distance
- Implemented brute-force nearest-neighbor search between current map-frame LiDAR points and saved-map reference points
- Used squared Euclidean distance during comparison to avoid unnecessary square-root calculations
- Verified correspondence count and mean nearest distance during stationary and rotation tests
- Observed mean nearest distances in the centimeter range while the map-frame scan remained aligned with the saved map

### Correspondence Filtering / Outlier Rejection
- Added a maximum correspondence-distance threshold
- Separated Raw, Valid, and Rejected correspondence counts
- Added Raw mean, Valid mean, and Raw maximum distance diagnostics
- Temporarily reduced the threshold to `0.05 m` to validate real outlier removal
- Verified `Raw = Valid + Rejected`
- Verified that removing distant correspondences reduces or preserves the valid mean distance
- Observed rejection examples including `358 / 2`, `355 / 5`, and `351 / 9` Valid / Rejected pairs from 360 Raw pairs
- Restored the normal development threshold to `0.15 m`

### 2D Rigid ICP Correction
- Calculated Source and Target centroids from valid correspondences
- Centered the two correspondence point sets around their centroids
- Calculated the incremental 2D rotation correction `delta_yaw`
- Calculated translation correction `delta_x` and `delta_y`
- Composed the incremental correction with the existing `map_to_odom_` estimate using `T_new = Delta_T * T_old`
- Recalculated map-frame scan points using the updated pose
- Added correction-before / correction-after mean-distance validation
- Verified an initial correction reduced mean correspondence distance from approximately `0.02343 m` to `0.01751 m`

### ICP Iteration / Convergence
- Changed the ICP flow from one correction per incoming scan to multiple iterations on one fixed LaserScan frame
- Added a maximum of `10` iterations
- Added translation convergence threshold of `0.001 m`
- Added rotation convergence threshold of `0.001 rad`
- Recomputed correspondences and rigid correction at each iteration
- Verified stationary convergence with `converged: true`
- Verified subsequent scans can converge at iteration `1/10` with correction values effectively equal to zero
- Observed a stable internal `map_to_odom_` estimate near `(0.01555, 0.02279, -0.00001)` during stationary validation

### ICP Localization Launch Integration
- Added `localization_icp.launch.py`
- Integrated benchmark simulation startup, Nav2 Map Server, lifecycle configure / activate, temporary `map -> odom` static TF, and custom ICP localization into one launch command
- Fixed a ROS2 Humble `LifecycleNode` startup error by explicitly setting `namespace=''`
- Verified the benchmark map loads as `199 x 198 @ 0.05 m/cell`
- Verified the ICP node receives the map and converts it to `1748` reference points

### Gazebo Simulation Clock Integration
- Identified that ROS2 `/clock` existed with `Publisher count: 0` while nodes were configured with `use_sim_time=true`
- Confirmed that the missing clock publisher prevented time-dependent throttled ICP logs from behaving normally
- Verified a temporary manual Gazebo-to-ROS clock bridge immediately restored the ICP iteration logs
- Added `/clock` to the existing ROS-Gazebo bridge YAML so a separate clock terminal is no longer required
- Kept Map Server and ICP localization on simulation time

### LaserScan Timestamp-Aligned TF
- Replaced `tf2::TimePointZero` with the current `LaserScan.header.stamp`
- Added a `0.1 s` TF lookup timeout
- Verified timestamp-synchronized `laser_link -> odom` TF lookup
- Removed the previous dependency on the latest available TF for scan transformation
- Prepared the ICP pipeline for more reliable localization while the robot is moving

### Current Milestone
- Nearest-Neighbor Correspondence Search: Completed
- Outlier Rejection: Completed
- Centroid Calculation: Completed
- 2D Rigid Correction: Completed
- Internal `map_to_odom_` Update: Completed
- ICP Iteration / Convergence: Completed
- Integrated ICP Launch: Completed
- Gazebo `/clock` Bridge: Completed
- LaserScan Timestamp-Aligned TF: Completed
- Dynamic ROS2 `map -> odom` TF Broadcast: Next

### Next Step
- Add a `tf2_ros::TransformBroadcaster` to the custom ICP node
- Publish the calculated `map_to_odom_` as the real dynamic `map -> odom` TF
- Remove the temporary static identity `map -> odom` publisher from `localization_icp.launch.py`
- Validate localization while the robot translates and rotates
- Begin ICP quantitative accuracy and runtime measurements

## 2026-09-21

### Dynamic `map -> odom` TF Broadcast
- Added `tf2_ros::TransformBroadcaster` to the custom ICP localization node
- Published the current ICP correction as the dynamic `map -> odom` transform
- Used the current LaserScan timestamp for the transform stamp
- Removed the temporary static identity `map -> odom` publisher from the localization launch
- Verified `map -> odom` updates at approximately the LiDAR / ICP update rate
- Verified the complete `map -> odom -> base_footprint` TF chain

### Moving-Robot ICP Validation
- Validated straight-line motion while Custom ICP continuously updated the map-relative correction
- Validated rotation while robot yaw changed in `odom -> base_footprint` and `map -> odom` remained a small correction
- Confirmed that `map -> base_footprint` represents the final map-relative robot pose

### Gazebo Ground Truth Integration
- Bridged `/world/localization_world/dynamic_pose/info` into ROS2 as `/ground_truth/poses`
- Added the `localization_evaluation` package
- Added `ground_truth_node` to extract the AMR pose and convert Gazebo world coordinates into the fixed benchmark start-relative coordinate system
- Published the evaluation reference as `/ground_truth_pose`
- Verified the benchmark start becomes approximately `(0, 0, 0)`

### Odom / ICP Evaluation Infrastructure
- Added `localization_evaluation_node`
- Compared Ground Truth against Wheel Odometry and the composed Custom ICP `map -> base_footprint` pose
- Added instantaneous position / yaw error and cumulative RMSE calculations
- Identified that short straight motion in the original Gazebo setup did not expose enough odometry drift for a meaningful benchmark

### Nav2 Navigation-Only Integration
- Added `nav2_icp_params.yaml`
- Added `navigation_icp.launch.py`
- Used Nav2 for global planning, local control, costmaps, obstacle avoidance, waypoint execution, and velocity smoothing
- Did not use AMCL in this benchmark; Custom ICP provides `map -> odom`
- Configured Global Costmap in `map` and rolling Local Costmap in `odom`
- Fixed a ROS2 Humble parameter-type error by changing Local Costmap `width` / `height` from `3.0` to integer `3`
- Verified all Nav2 managed nodes reach the ACTIVE lifecycle state
- Verified manual `NavigateToPose` navigation to a test point

### Automated Waypoint Benchmark
- Added `localization_benchmark_runner`
- Added `benchmark_icp.launch.py` to start simulation, Custom ICP, Map Server, Nav2, Ground Truth, and benchmark execution together
- Added Nav2 lifecycle-state gating so the first waypoint is not sent before `bt_navigator` becomes ACTIVE
- Disabled temporary readiness/debug logs after the startup sequence was validated
- Configured the final benchmark route:
  - `P1 = (3.39557, -4.12722, 0.0)`
  - `P2 = (5.61326, 4.40286, 0.0)`
  - `P3 = (7.68631, -1.58174, 0.0)`
- Selected the route to make turning, longer travel, and obstacle avoidance visible
- Recorded continuous Ground Truth / Odom / ICP samples and waypoint snapshots into CSV
- Finished measurement automatically after P3 succeeds

### First Odom vs ICP Benchmark Result
- Recorded one complete START -> P1 -> P2 -> P3 simulation run
- Overall Wheel Odometry Position RMSE: `0.1149 m`
- Overall Custom ICP Position RMSE: `0.0428 m`
- Overall Wheel Odometry Yaw RMSE: `0.0179 rad`
- Overall Custom ICP Yaw RMSE: `0.0123 rad`
- Observed approximately `62.7%` lower overall Position RMSE with Custom ICP in this run
- Segment Position RMSE:
  - START -> P1: Odom `0.0391 m`, ICP `0.0351 m`
  - P1 -> P2: Odom `0.1121 m`, ICP `0.0321 m`
  - P2 -> P3: Odom `0.1636 m`, ICP `0.0604 m`
- Observed increasing Wheel Odometry position drift along the longer route while scan-to-map ICP limited the growth of the map-relative position error
- Confirmed that ICP still retains residual error and does not force localization error to zero

### Benchmark Artifacts
- Added combined Gazebo + RViz2 benchmark visualization as `docs/images/14_nav2_icp_gazebo_rviz_benchmark.gif`
- Added checked-in representative CSV as `results/benchmark_01/localization_benchmark.csv`
- Added `results/benchmark_01/trajectory_comparison.png`
- Added `results/benchmark_01/error_analysis.png`
- Kept quantitative benchmark outputs separate from `docs/images/`

### Current Evaluation Limitations
- Current quantitative values are from a single simulation run
- Continuous evaluator samples currently compare the latest available Ground Truth, Odometry, and ICP-composed TF values rather than a final offline strict timestamp alignment
- Repeated trials and statistical summary are not yet complete
- ICP processing time / CPU usage are not yet included in the benchmark

### Current Milestone
- Dynamic Custom ICP TF: Completed
- Moving-Robot ICP Validation: Completed
- Gazebo Ground Truth: Completed
- Nav2 Navigation-Only Integration: Completed
- Automated Waypoint Benchmark: Completed
- Odom vs ICP Quantitative Baseline: Completed
- Result CSV / Trajectory / Error Graphs: Completed
- EKF Sensor Fusion: Next

### Next Step
- Implement the EKF state / prediction model in C++
- Fuse Wheel Odometry, Custom ICP, and IMU
- Add ICP+EKF as a third estimator to the same automated benchmark
- Repeat benchmark runs and calculate mean / standard deviation
- Add strict timestamp-aligned evaluation and runtime / CPU measurements

