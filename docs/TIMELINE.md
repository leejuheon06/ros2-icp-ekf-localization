# Development Timeline

## 2026-09-04

### AMR URDF Model
- Created differential-drive AMR Xacro model
- Added `base_link` and `base_footprint`
- Added left/right drive wheels
- Added LiDAR frame
- Added IMU frame
- Verified RobotModel in RViz2
- Verified TF tree

### Gazebo Simulation
- Created `empty_world.sdf`
- Added ground plane
- Spawned AMR into Gazebo Fortress
- Identified pitch instability in the initial two-wheel configuration
- Added front/rear passive spherical casters
- Configured low-friction caster contacts

### Issues Resolved
- Fixed unstable two-wheel ground contact

---

## 2026-09-05 ~ 2026-09-19

### Differential Drive / ROS2 Bridge
- Added Gazebo Differential Drive
- Verified forward, backward, and in-place rotation
- Fixed reversed motion by correcting wheel joint axis direction
- Integrated `ros_gz_bridge`
- Verified ROS2 `/cmd_vel`
- Verified wheel odometry `/odom`
- Integrated Gazebo wheel joint states with ROS2 `/joint_states`

### Sensor Simulation
- Added 360-degree 2D LiDAR
- Bridged Gazebo LaserScan to ROS2 `/scan`
- Added IMU sensor and ROS2 `/imu`
- Verified sensor frames and RViz2 visualization

### Mapping / Evaluation World
- Created `localization_world.sdf`
- Selected fixed benchmark start pose
- Created and saved `localization_map.pgm/.yaml`
- Verified saved-map reload
- Prepared map QoS and TF environment for localization

---

## 2026-09-20

### Custom ICP Localization
- Created `icp_localization` ROS2 C++ package
- Implemented LaserScan -> 2D point conversion
- Implemented OccupancyGrid -> persistent map reference points
- Implemented `laser_link -> odom -> map` point transformation
- Added brute-force nearest-neighbor correspondence search
- Added maximum correspondence-distance filtering
- Implemented source/target centroid calculation
- Implemented 2D rigid correction for `delta_x`, `delta_y`, `delta_yaw`
- Added iterative ICP update and convergence condition
- Bridged Gazebo `/clock` to ROS2 `/clock`
- Changed TF lookup to LaserScan timestamp-based lookup
- Integrated custom ICP launch

### Validation
- Verified map target and transformed scan source in RViz2
- Verified translation and rotation behavior

---

## 2026-09-21

### Dynamic ICP Localization
- Replaced temporary static `map -> odom` with dynamic ICP TF broadcast
- Verified `map -> odom -> base_footprint` TF structure
- Added `/icp_pose` as the global robot pose in the `map` frame

### Ground Truth / Nav2 Benchmark
- Added `localization_evaluation` package
- Added Gazebo Ground Truth conversion
- Integrated Nav2 planning and control without AMCL
- Created automated `START -> P1 -> P2 -> P3` benchmark
- Added CSV logging and Odom/ICP error calculation
- Completed initial Odom-vs-ICP quantitative baseline

### EKF Sensor Fusion
- Added `ekf_localization` C++ package
- Defined EKF state `[x, y, yaw]`
- Implemented prediction with wheel odometry linear velocity and IMU yaw rate
- Implemented covariance propagation
- Added ICP pose measurement update
- Added innovation, Kalman gain, angle normalization, and Joseph covariance update
- Published `/ekf_pose` and `/ekf_odom`
- Verified EKF initialization and motion updates

### Benchmark Integration
- Extended benchmark CSV to Ground Truth / Odom / ICP / EKF
- Added EKF position and yaw error calculation
- Added Odom / ICP / EKF RMSE summary
- Staggered launch startup to reduce Nav2 lifecycle contention
- Reduced repetitive ICP / EKF runtime logs while preserving warnings and errors

---

## 2026-09-22

### Final Repeated Benchmark
- Executed four repeated runs using the same START -> P1 -> P2 -> P3 route
- Saved all four benchmark CSV files under `results/benchmark_02/`
- Generated trajectory, time-series error, RMSE, and segment comparison graphs

### Final Quantitative Result

Four-run mean ± sample standard deviation:

| Method | Position RMSE | Yaw RMSE |
|---|---:|---:|
| Wheel Odometry | `0.0704 ± 0.0108 m` | `0.01106 ± 0.00196 rad` |
| Custom ICP | `0.04288 ± 0.00022 m` | `0.01279 ± 0.00016 rad` |
| Custom ICP + EKF | `0.04211 ± 0.00021 m` | `0.00510 ± 0.00010 rad` |

### Result Interpretation
- Custom ICP reduced mean Position RMSE by approximately `39.1%` compared with Wheel Odometry
- ICP + EKF reduced mean Position RMSE by approximately `40.1%` compared with Wheel Odometry
- ICP -> EKF reduced mean Yaw RMSE by approximately `60.1%`
- Most position-drift reduction came from map-based ICP correction
- EKF provided the clearest improvement in yaw / heading stability
- ICP and EKF Position RMSE showed low run-to-run variation under the repeated simulation benchmark

### Project Milestone
- AMR simulation completed
- Sensor integration completed
- Custom ICP completed
- Custom EKF completed
- Nav2 benchmark automation completed
- Ground Truth evaluation completed
- Four-run quantitative evaluation completed
- Result graphs and documentation completed

### Current Status

**project scope completed.**

Optional future extensions:
- EKF-owned `map -> odom` TF and Nav2 navigation directly from the fused pose
- Strict timestamp interpolation / rosbag offline evaluation
- Mahalanobis gating for ICP measurement rejection
- Mean / P95 processing-time and CPU measurement
- AMCL / SLAM Toolbox localization comparison
- Physical robot validation
