# ROS2 ICP + EKF Localization — Differential-Drive AMR

<div align="right">

[한국어](README_KO.md) | [English](README.md)

</div>

**Scan-to-Map ICP · Wheel Odometry · IMU Fusion · Nav2 · Ground Truth Benchmark**  
ROS2 Humble / C++17 환경에서 구현하고 정량 평가한 Differential-Drive AMR Localization 프로젝트입니다.

---

## 프로젝트 개요

이 프로젝트는 **2D LiDAR, Wheel Odometry, IMU, Custom ICP Scan Matching, Custom Extended Kalman Filter(EKF)** 를 이용해 AMR(Autonomous Mobile Robot)의 위치추정 파이프라인을 직접 구현한 프로젝트입니다.

목표는 기존 localization 패키지를 단순히 실행하는 것이 아니라, 위치추정 내부 과정을 직접 구현하고 각 단계의 성능을 정량적으로 비교하는 것입니다.

```text
Wheel Odometry
      +
2D LiDAR Scan-to-Map ICP
      +
IMU
      ↓
Custom EKF
      ↓
[x, y, yaw]
```

시스템은 Nav2와 통합되어 있으며, Gazebo Ground Truth를 기준으로 자동화된 **START → P1 → P2 → P3** 경로를 주행하면서 성능을 평가합니다.

**핵심 구현 내용**

- C++17 기반 **Custom 2D Scan-to-Map ICP**
- ICP가 동적으로 발행하는 **`map -> odom`** correction
- 상태 `[x, y, yaw]` 기반 **Custom EKF**
- Wheel Odometry + IMU yaw rate 기반 EKF Prediction
- ICP global pose 기반 EKF Measurement Update
- **AMCL 없이** Nav2 Navigation 통합
- Gazebo Ground Truth 기반 자동 benchmark 및 CSV logging
- 동일 경로 / 동일 조건 **4회 반복 주행**
- 최종 평균 Position RMSE
  - Wheel Odometry: **0.0704 m**
  - ICP: **0.04288 m**
  - ICP + EKF: **0.04211 m**
- EKF 적용 후 ICP 대비 Yaw RMSE 약 **60.1% 감소**

---

## 주요 결과

최종 benchmark 결과 이미지는 `results/benchmark_02/`에서 직접 불러옵니다.

<table>
<tr>
<td width="50%" align="center">
<b>Trajectory Comparison</b><br><br>
<img src="results/benchmark_02/trajectory_comparison.png" width="100%">
</td>
<td width="50%" align="center">
<b>Error Analysis</b><br><br>
<img src="results/benchmark_02/error_analysis.png" width="100%">
</td>
</tr>
<tr>
<td width="50%" align="center">
<b>4회 평균 Position RMSE</b><br><br>
<img src="results/benchmark_02/4run_mean_position_rmse.png" width="100%">
</td>
<td width="50%" align="center">
<b>4회 평균 Yaw RMSE</b><br><br>
<img src="results/benchmark_02/4run_mean_yaw_rmse.png" width="100%">
</td>
</tr>
</table>

추가 상세 그래프:

- [Position Error Over Time](results/benchmark_02/position_error_time.png)
- [Yaw Error Over Time](results/benchmark_02/yaw_error_time.png)
- [Position RMSE Comparison](results/benchmark_02/position_rmse_comparison.png)
- [Yaw RMSE Comparison](results/benchmark_02/yaw_rmse_comparison.png)
- [Segment Position RMSE](results/benchmark_02/segment_position_rmse.png)
- [Segment Yaw RMSE](results/benchmark_02/segment_yaw_rmse.png)

---

## 시스템 아키텍처

> *Simulation → Localization → Fusion → Navigation → Evaluation*

```text
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo Fortress                         │
│                                                             │
│   DiffDrive        2D LiDAR          IMU       Ground Truth │
└──────┬───────────────┬───────────────┬──────────────┬───────┘
       │               │               │              │
       v               v               v              v
    /odom            /scan           /imu    /ground_truth_pose
       │               │               │              │
       │        ┌──────▼────────┐      │              │
       │        │  Custom ICP   │      │              │
       │        │ Scan-to-Map   │      │              │
       │        └──────┬────────┘      │              │
       │               │               │              │
       │       map -> odom TF           │              │
       │               │               │              │
       │          /icp_pose             │              │
       │               │               │              │
       └───────────────┬┴───────────────┘              │
                       v                               │
                ┌───────────────┐                      │
                │  Custom EKF   │                      │
                │ [x, y, yaw]   │                      │
                └──────┬────────┘                      │
                       │                               │
                 /ekf_pose                             │
                 /ekf_odom                             │
                       │                               │
         ┌─────────────┴─────────────┐                 │
         │                           │                 │
         v                           v                 │
      Nav2                     Benchmark Runner <──────┘
 Planning / Control          Odom / ICP / EKF Error
         │                           │
         v                           v
     /cmd_vel                 CSV / RMSE / Plots
```

### 검증된 TF 구조

```text
map
 |
 |  Custom ICP
 v
odom
 |
 |  Wheel Odometry
 v
base_footprint
 |
 v
base_link
 ├── laser_link
 ├── imu_link
 ├── left_wheel_link
 └── right_wheel_link
```

> **현재 benchmark 구조:** Nav2는 ICP가 발행하는 `map -> odom` TF를 사용합니다.  
> EKF는 `/ekf_pose`를 통해 병렬로 정량 평가되며, 아직 최종 TF authority는 아닙니다.

---

## 주요 기능

### Localization — Custom 2D Scan-to-Map ICP

ICP node는 현재 LiDAR scan과 저장된 Occupancy Grid에서 추출한 map point들을 정합합니다.

```text
LaserScan
   ↓
2D Scan Points
   ↓
laser_link -> odom
   ↓
odom -> map
   ↓
Nearest-Neighbor Search
   ↓
Correspondence Filtering
   ↓
2D Rigid Transform
   ↓
Iterative Update
   ↓
map -> odom
```

**구현 파라미터**

| 항목 | 값 |
|---|---:|
| LiDAR sample 수 | 360 |
| LiDAR update rate | 10 Hz |
| Scan range | 0.10–10.0 m |
| Nearest-neighbor 방식 | Brute-force |
| Max correspondence distance | 0.15 m |
| 최대 ICP iteration | 10 |
| Translation convergence | 0.001 m |
| Rotation convergence | 0.001 rad |

Matching된 Source / Target point pair로부터 2D rigid correction을 계산하고, 이를 내부 `map_to_odom` transform에 반복적으로 합성합니다.

---

### Sensor Fusion — Custom EKF

EKF state는 다음과 같습니다.

```text
[x, y, yaw]
```

**Prediction 입력**

```text
Wheel Odometry
- linear.x

IMU
- angular_velocity.z
```

**Measurement 입력**

```text
Custom ICP
- global x
- global y
- global yaw
```

Prediction model:

```text
x(k+1)   = x(k)   + v cos(yaw) dt
y(k+1)   = y(k)   + v sin(yaw) dt
yaw(k+1) = yaw(k) + omega dt
```

구현 내용:

- State prediction
- Jacobian 기반 covariance propagation
- ICP innovation 계산
- Yaw residual normalization
- Kalman gain 계산
- Measurement update
- Joseph-form covariance update

초기 tuning 값:

| Parameter | Value |
|---|---:|
| Process noise XY | 0.02 |
| Process noise yaw | 0.02 |
| Initial std XY | 0.05 |
| Initial std yaw | 0.03 |
| ICP measurement std XY | 0.05 |
| ICP measurement std yaw | 0.03 |

---

### Navigation — Custom Localization + Nav2

Nav2는 다음 기능을 담당합니다.

- Global planning
- Local costmap
- Obstacle avoidance
- DWB local control
- `NavigateToPose`

Benchmark에서는 AMCL을 사용하지 않습니다.

```text
Custom ICP
    ↓
map -> odom
    ↓
Nav2
    ↓
/cmd_vel
    ↓
Gazebo AMR
```

---

## 구현 및 검증 자료

`docs/images/`의 자료는 구현 과정과 검증 내용을 기록한 이미지 / GIF입니다.  
`results/benchmark_02/`는 최종 정량 평가 결과 전용으로 사용합니다.

### Robot Model & Differential Drive

<table>
<tr>
<td width="50%" align="center">
<b>RViz2 Robot Model 검증</b><br><br>
<img src="docs/images/01_robot_model.png" width="100%">
</td>
<td width="50%" align="center">
<b>Differential Drive — 직진</b><br><br>
<img src="docs/images/02_differential_drive_gazebo_straight.gif" width="100%">
</td>
</tr>
<tr>
<td width="50%" align="center">
<b>Differential Drive — 회전</b><br><br>
<img src="docs/images/02_differential_drive_gazebo_turn.gif" width="100%">
</td>
<td width="50%" align="center">
<b>ROS2 ↔ Gazebo Bridge</b><br><br>
<img src="docs/images/03_differential_drive_ROS2_bridge.gif" width="100%">
</td>
</tr>
</table>

### Joint State & LiDAR 검증

![Joint States and LiDAR Scan in RViz2](docs/images/04_joint_states_lidar_scan_rviz.gif)

Gazebo의 wheel joint state와 LiDAR scan이 ROS2로 정상 전달되고 RViz2에서 시각화되는지 검증한 단계입니다.

### Evaluation World & Mapping

<table>
<tr>
<td width="50%" align="center">
<b>Localization Benchmark World</b><br><br>
<img src="docs/images/07_localization_world_gazebo.png.png" width="100%">
</td>
<td width="50%" align="center">
<b>SLAM Toolbox Mapping</b><br><br>
<img src="docs/images/08_slam_toolbox_mapping_rviz.gif" width="100%">
</td>
</tr>
<tr>
<td width="50%" align="center">
<b>생성된 Occupancy Grid</b><br><br>
<img src="docs/images/08_slam_toolbox_generated_map_rviz.png" width="100%">
</td>
<td width="50%" align="center">
<b>Saved Map Reload 검증</b><br><br>
<img src="docs/images/09_saved_map_reload_rviz.png" width="100%">
</td>
</tr>
</table>

Benchmark 환경은 Scan-to-Map matching 시 기하학적 모호성을 줄이기 위해 비대칭 형태의 벽과 장애물을 배치했습니다.

### ICP Input Preparation & Frame 검증

<table>
<tr>
<td width="50%" align="center">
<b>LaserScan → PointCloud 검증</b><br><br>
<img src="docs/images/10_laserscan_pointcloud_overlap_rviz.gif" width="100%">
</td>
<td width="50%" align="center">
<b>OccupancyGrid → Reference PointCloud</b><br><br>
<img src="docs/images/11_occupancygrid_reference_pointcloud_rviz.gif" width="100%">
</td>
</tr>
<tr>
<td width="50%" align="center">
<b>LaserScan → Odom Frame</b><br><br>
<img src="docs/images/12_laserscan_odom_transform_rviz.gif" width="100%">
</td>
<td width="50%" align="center">
<b>Map-Frame ICP Source / Target Alignment</b><br><br>
<img src="docs/images/13_icp_map_scan_alignment_rviz.png" width="100%">
</td>
</tr>
</table>

ICP iterative correction 이전에 다음 입력 pipeline을 검증했습니다.

```text
/scan
  ↓
2D scan points
  ↓
laser_link → odom
  ↓
odom → map
  ↓
Current scan [Source]
        +
Saved map points [Target]
```

### Nav2 + ICP 자동 Benchmark

![Nav2 + ICP Gazebo / RViz2 Benchmark](docs/images/14_nav2_icp_gazebo_rviz_benchmark.gif)

Nav2가 AMR을 주행시키는 동안 Custom ICP가 global `map -> odom` correction을 제공하는 통합 benchmark입니다.

---

## 자동 Benchmark

### 주행 경로

모든 최종 실험은 동일한 경로를 사용합니다.

```text
START
  |
  v
P1  (3.39557, -4.12722, 0.0)
  |
  v
P2  (5.61326,  4.40286, 0.0)
  |
  v
P3  (7.68631, -1.58174, 0.0)
```

### Benchmark 입력

```text
/ground_truth_pose
/odom
/icp_pose
/ekf_pose
```

### 기록 데이터

각 CSV에는 다음 값이 기록됩니다.

```text
Ground Truth x / y / yaw
Wheel Odometry x / y / yaw
ICP x / y / yaw
EKF x / y / yaw

Odom position / yaw error
ICP position / yaw error
EKF position / yaw error

START / P1 / P2 / P3 event
continuous SAMPLE rows
```

### Startup 순서

Nav2 lifecycle startup 경합을 줄이기 위해 실행 순서를 나누었습니다.

```text
0 s   Gazebo + ICP + Nav2
10 s  EKF
12 s  Ground Truth
15 s  Benchmark Runner
```

Benchmark Runner는 첫 goal을 보내기 전에 `bt_navigator`가 ACTIVE 상태인지 추가로 확인합니다.

---

## 성능 결과

### 4회 반복 정량 평가

| Method | Position RMSE | Yaw RMSE |
|---|---:|---:|
| Wheel Odometry | **0.0704 ± 0.0108 m** | **0.01106 ± 0.00196 rad** |
| Custom ICP | **0.04288 ± 0.00022 m** | **0.01279 ± 0.00016 rad** |
| Custom ICP + EKF | **0.04211 ± 0.00021 m** | **0.00510 ± 0.00010 rad** |

### 개선율

| 비교 | 결과 |
|---|---:|
| Odom → ICP Position RMSE | **39.1% 감소** |
| Odom → ICP+EKF Position RMSE | **40.1% 감소** |
| ICP → ICP+EKF Position RMSE | **1.8% 감소** |
| ICP → ICP+EKF Yaw RMSE | **60.1% 감소** |

### Position RMSE

```text
Wheel Odometry
7.04 ± 1.08 cm

        ↓  ICP

Custom ICP
4.29 ± 0.02 cm

        ↓  EKF

ICP + EKF
4.21 ± 0.02 cm
```

Position drift 감소의 대부분은 **ICP 단계에서 발생**했습니다.

### Yaw RMSE

```text
Wheel Odometry
0.01106 rad

Custom ICP
0.01279 rad

        ↓ EKF Fusion

ICP + EKF
0.00510 rad
```

EKF의 가장 뚜렷한 효과는 **heading / yaw 안정화**에서 나타났습니다.

---

## 결과 그래프

아래 이미지는 repository-relative path를 사용하므로 GitHub README에서 직접 표시됩니다.

### Trajectory & Overall Error

![Trajectory Comparison](results/benchmark_02/trajectory_comparison.png)

![Error Analysis](results/benchmark_02/error_analysis.png)

### Error Over Time

![Position Error Over Time](results/benchmark_02/position_error_time.png)

![Yaw Error Over Time](results/benchmark_02/yaw_error_time.png)

### Overall RMSE

![Position RMSE Comparison](results/benchmark_02/position_rmse_comparison.png)

![Yaw RMSE Comparison](results/benchmark_02/yaw_rmse_comparison.png)

### Segment-Level RMSE

![Segment Position RMSE](results/benchmark_02/segment_position_rmse.png)

![Segment Yaw RMSE](results/benchmark_02/segment_yaw_rmse.png)

### 4회 평균 ± 표준편차

![4-Run Mean Position RMSE](results/benchmark_02/4run_mean_position_rmse.png)

![4-Run Mean Yaw RMSE](results/benchmark_02/4run_mean_yaw_rmse.png)

---

## 주요 결과 및 설계 판단

### 1. Position drift 감소의 핵심은 ICP

Wheel Odometry는 주행이 진행될수록 누적 Position error가 증가했습니다.

ICP 적용 후 4회 평균 Position RMSE:

```text
0.0704 m
   ↓
0.04288 m
```

저장된 map이 global reference 역할을 하기 때문에 Odom의 누적 drift를 제한할 수 있었습니다.

---

### 2. EKF의 가장 큰 효과는 Yaw 안정화

EKF 적용 전후 Position RMSE 차이는 크지 않았습니다.

```text
ICP       0.04288 m
ICP+EKF   0.04211 m
```

반면 Yaw RMSE는 크게 감소했습니다.

```text
ICP       0.01279 rad
ICP+EKF   0.00510 rad
```

현재 fusion 구조에서는 EKF가 x/y를 크게 추가 보정하기보다 orientation을 안정화하는 데 더 효과적이라는 결과를 확인했습니다.

---

### 3. ICP가 모든 상태에서 항상 Odom보다 정확한 것은 아님

4회 평균 기준 ICP Yaw RMSE는 Wheel Odometry보다 약간 크게 나타났습니다.

```text
ICP ≠ 모든 상태 변수에서 자동으로 더 정확함
```

현재 결과에서는:

```text
ICP
→ Position correction에 강점

EKF
→ Yaw stabilization에 강점
```

이라는 역할 차이가 확인됩니다.

---

### 4. TF 구조와 알고리즘 실행 순서는 다름

TF tree:

```text
map -> odom -> base_footprint
```

실제 처리 순서:

```text
Robot moves
    ↓
Wheel odometry update
    ↓
LiDAR scan arrives
    ↓
ICP scan-to-map alignment
    ↓
map -> odom correction update
```

TF hierarchy는 좌표계 간 관계를 의미하며, 알고리즘의 시간적 실행 순서를 의미하지 않습니다.

---

### 5. Moving ICP에는 timestamp-aligned TF가 필요함

Latest TF만 사용하면 로봇이 이동 중일 때 LiDAR measurement time과 TF time이 달라질 수 있습니다.

따라서 ICP node는 LaserScan timestamp 기준으로 TF lookup을 수행하도록 변경했습니다.

이 과정은 이동 중 scan placement와 robot pose 사이의 temporal consistency를 확보하기 위해 필요했습니다.

---

### 6. Nav2 startup은 단계적으로 실행

Gazebo, Nav2 lifecycle nodes, ICP, EKF, Ground Truth, Benchmark Runner를 동시에 시작했을 때 `smoother_server` lifecycle service timeout이 발생했습니다.

최종 launch에서는 다음 순서로 실행합니다.

```text
Navigation → EKF → Ground Truth → Benchmark Runner
```

이후 `bt_navigator` ACTIVE 상태를 확인한 뒤 waypoint navigation을 시작하도록 구성했습니다.

---

## 주요 문제 해결 기록

| 문제 | 원인 | 해결 |
|---|---|---|
| `/cmd_vel` 양수에서 로봇이 반대로 이동 | Wheel joint axis 방향 반대 | Wheel joint axis를 `0 0 -1`로 수정 |
| RViz에서 wheel이 사라짐 | Continuous wheel joint state 미발행 | Gazebo JointStatePublisher 추가 및 `/joint_states` bridge |
| RViz LaserScan message drop | Message Filter / queue 문제 | Scan / TF 검증 설정 조정 |
| Static `map -> odom` 때문에 실제 correction 불가 | 검증용 static TF가 남아 있음 | Dynamic ICP TransformBroadcaster로 교체 |
| 이동 중 ICP temporal mismatch | Scan과 TF의 timestamp 불일치 | LaserScan timestamp 기반 TF lookup |
| Nav2 benchmark 미시작 | Lifecycle startup contention / smoother timeout | Benchmark launch 순차 실행 |
| ICP/EKF 반복 로그로 benchmark event 확인 어려움 | Per-update INFO logging | 반복 INFO 주석 처리, WARN / ERROR 유지 |

---

## 평가 조건

### Ground Truth

Gazebo Ground Truth는 **평가용 기준값으로만 사용**합니다.

```text
Ground Truth
      X
      |
      |  estimator input으로 사용하지 않음
      |
ICP / EKF
```

### 반복성

동일한 경로와 simulation configuration으로 총 4회 반복 주행했습니다.

Position RMSE 표준편차:

```text
Wheel Odometry : ± 0.0108 m
Custom ICP     : ± 0.00022 m
ICP + EKF      : ± 0.00021 m
```

현재 simulation benchmark에서 ICP / ICP+EKF 결과는 Wheel Odometry보다 주행 간 변동이 작았습니다.

---

## 한계

- 최종 평가는 Gazebo simulation 기반
- Benchmark sample은 모든 estimator를 하나의 정확한 timestamp로 offline interpolation하지 않음
- ICP와 EKF는 모두 Odometry 정보에 의존하므로 완전히 statistical independent하지 않음
- 현재 Nav2가 사용하는 최종 `map -> odom` TF는 ICP가 발행하며, EKF는 아직 최종 TF authority가 아님
- CPU utilization / processing-time 통계 미포함
- AMCL / SLAM Toolbox localization과 최종 4회 비교는 수행하지 않음
- ICP nearest-neighbor search는 현재 brute-force 방식이며 spatial index를 사용하지 않음

---

## Build & Run

### 전체 Workspace Build

```bash
cd ~/ros2_icp_ekf_localization

colcon build --symlink-install

source install/setup.bash
```

### Localization / Evaluation Package Build

```bash
colcon build \
  --symlink-install \
  --packages-select \
  icp_localization \
  ekf_localization \
  localization_evaluation \
  robot_simulation

source install/setup.bash
```

### Robot Model 검증

```bash
ros2 launch robot_description display.launch.py
```

### Gazebo Simulation

```bash
ros2 launch robot_simulation simulation.launch.py
```

### Custom ICP Localization

```bash
ros2 launch robot_simulation localization_icp.launch.py
```

### Custom ICP + Nav2

```bash
ros2 launch robot_simulation navigation_icp.launch.py
```

### Automated Odom / ICP / EKF Benchmark

```bash
ros2 launch robot_simulation benchmark_icp.launch.py
```

---

## Repository 구조

```text
ros2_icp_ekf_localization/
├── src/
│   ├── robot_description/
│   │   ├── launch/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── urdf/amr.urdf.xacro
│   │
│   ├── robot_simulation/
│   │   ├── config/
│   │   │   ├── bridge.yaml
│   │   │   ├── slam_toolbox.yaml
│   │   │   └── nav2_icp_params.yaml
│   │   ├── launch/
│   │   │   ├── simulation.launch.py
│   │   │   ├── localization_icp.launch.py
│   │   │   ├── navigation_icp.launch.py
│   │   │   └── benchmark_icp.launch.py
│   │   └── worlds/
│   │
│   ├── icp_localization/
│   │   └── src/icp_localization_node.cpp
│   │
│   ├── ekf_localization/
│   │   ├── config/
│   │   ├── launch/
│   │   └── src/ekf_localization_node.cpp
│   │
│   └── localization_evaluation/
│       └── src/
│           ├── ground_truth_node.cpp
│           ├── localization_evaluation_node.cpp
│           └── localization_benchmark_runner.cpp
│
├── maps/
│   ├── localization_map.pgm
│   └── localization_map.yaml
│
├── results/
│   ├── benchmark_01/
│   └── benchmark_02/
│
├── docs/
│   ├── COMMANDS.md
│   ├── TIMELINE.md
│   └── images/
│
├── README.md
└── .gitignore
```

---

## 최종 KPI 요약

| KPI | Result |
|---|---:|
| 반복 Benchmark 횟수 | **4회** |
| Wheel Odometry Position RMSE | **0.0704 ± 0.0108 m** |
| ICP Position RMSE | **0.04288 ± 0.00022 m** |
| ICP + EKF Position RMSE | **0.04211 ± 0.00021 m** |
| Odom → ICP Position 개선율 | **39.1%** |
| Odom → ICP+EKF Position 개선율 | **40.1%** |
| ICP Yaw RMSE | **0.01279 ± 0.00016 rad** |
| ICP + EKF Yaw RMSE | **0.00510 ± 0.00010 rad** |
| ICP → EKF Yaw 개선율 | **60.1%** |

---

## Tech Stack

**Framework:** ROS2 Humble  
**Language:** C++17  
**Simulation:** Gazebo Fortress  
**Visualization:** RViz2  
**Navigation:** Nav2  
**Localization:** Custom 2D ICP  
**Sensor Fusion:** Custom EKF  
**Sensors:** 2D LiDAR, Wheel Odometry, IMU  
**Bridge:** ros_gz_bridge  
**Build:** colcon / CMake  

---

## 결론

이 프로젝트에서는 AMR localization 개발 과정을 처음부터 정량 평가까지 연결했습니다.

```text
AMR Model
   ↓
Gazebo Simulation
   ↓
Sensor Integration
   ↓
Saved Map
   ↓
Custom ICP
   ↓
Custom EKF
   ↓
Nav2 Integration
   ↓
Ground Truth Benchmark
   ↓
4회 반복 정량 평가
```

최종 결과:

- **ICP는 Wheel Odometry의 누적 Position drift를 효과적으로 억제**
- **Position 개선의 대부분은 ICP 단계에서 발생**
- **EKF는 Yaw / Heading 안정화에서 가장 큰 효과**
- **ICP와 ICP+EKF는 동일 simulation benchmark에서 반복적으로 유사한 성능을 확인**

현재 구현은 포트폴리오에서 목표로 한 범위까지 완료되었습니다.

---

## 향후 개선

- EKF를 최종 `map -> odom` publisher로 변경
- Nav2가 fused EKF localization을 직접 사용하도록 통합
- ICP measurement rejection을 위한 Mahalanobis gating 추가
- Strict timestamp-aligned rosbag offline evaluation
- ICP / EKF mean / P95 processing time 측정
- AMCL / SLAM Toolbox localization과 정량 비교
- Brute-force nearest-neighbor를 KD-tree 또는 spatial index로 개선
- 실제 AMR hardware에서 검증

---

## 결과 파일

```text
results/benchmark_02/
├── 4run_mean_position_rmse.png
├── 4run_mean_yaw_rmse.png
├── error_analysis.png
├── localization_benchmark_1.csv
├── localization_benchmark_2.csv
├── localization_benchmark_3.csv
├── localization_benchmark_4.csv
├── position_error_time.png
├── position_rmse_comparison.png
├── segment_position_rmse.png
├── segment_yaw_rmse.png
├── trajectory_comparison.png
├── yaw_error_time.png
└── yaw_rmse_comparison.png
```

## 문서

이전 개발 과정 중심 README는 다음 위치에 보관할 수 있습니다.

```text
docs/README_DEVELOPMENT.md
```

실행 / 검증 명령어:

```text
docs/COMMANDS.md
```

개발 단계별 이력:

```text
docs/TIMELINE.md
```