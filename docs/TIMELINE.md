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
