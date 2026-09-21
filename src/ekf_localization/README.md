# ekf_localization

Custom EKF localization package for the ROS2 ICP + EKF AMR project.

## Current milestone

This version implements both EKF prediction and ICP measurement update.

### Prediction

Inputs:

- `/odom`
  - `twist.twist.linear.x` -> linear velocity
- `/imu`
  - `angular_velocity.z` -> yaw rate

State:

```text
[x, y, yaw]
```

Motion model:

```text
x'   = x + v * cos(yaw) * dt
y'   = y + v * sin(yaw) * dt
yaw' = yaw + omega * dt
```

Covariance prediction:

```text
P' = F * P * F^T + Q
```

### Measurement update

Input:

- `/icp_pose`
  - Custom ICP map-frame robot pose

Measurement:

```text
z = [x_icp, y_icp, yaw_icp]
```

For the current 3-state model:

```text
H = I
innovation = z - x
S = P + R
K = P * S^-1
x = x + K * innovation
```

Joseph-form covariance update is used after the state correction.

### Outputs

- `/ekf_pose`
- `/ekf_odom`

The EKF still does not publish `map -> odom` TF at this stage. Custom ICP remains the TF publisher while the fused estimate is validated independently.