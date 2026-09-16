# ROS2 ICP + EKF Localization --- 실행 명령어 정리

> 개발 환경: Ubuntu 22.04 LTS / ROS2 Humble / Gazebo Fortress\
> Workspace: `~/ros2_icp_ekf_localization`\
> 작성일: 2026-09-04

이 문서는 프로젝트 초기 구성부터 RViz의 URDF/TF 확인, Gazebo World 실행
및 AMR Spawn까지 진행하면서 사용한 주요 명령어를 정리한 개발 노트입니다.

------------------------------------------------------------------------

## 1. ROS2 환경 설정

새 터미널을 열었을 때 ROS2 Humble 환경을 불러옵니다.

``` bash
source /opt/ros/humble/setup.bash
```

Workspace가 이미 빌드되어 있다면 프로젝트 환경도 불러옵니다.

``` bash
cd ~/ros2_icp_ekf_localization
source install/setup.bash
```

매번 입력하기 번거롭다면 `~/.bashrc`에 ROS2 Humble source를 추가할 수
있습니다.

``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

------------------------------------------------------------------------

## 2. Workspace 이동 및 구조 확인

프로젝트 루트로 이동합니다.

``` bash
cd ~/ros2_icp_ekf_localization
```

현재 디렉터리 구조를 확인합니다.

``` bash
tree -L 3
```

`tree`가 설치되어 있지 않다면:

``` bash
sudo apt install tree
```

현재 프로젝트의 주요 구조는 다음과 같습니다.

``` text
ros2_icp_ekf_localization/
├── src/
│   ├── robot_description/
│   │   ├── launch/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── urdf/
│   └── robot_simulation/
│       ├── config/
│       ├── launch/
│       └── worlds/
├── build/
├── install/
└── log/
```

------------------------------------------------------------------------

## 3. robot_description 빌드

URDF/Xacro 또는 launch 파일을 수정한 후 패키지를 다시 빌드합니다.

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select robot_description
```

빌드 후 반드시 새 환경을 source 합니다.

``` bash
source install/setup.bash
```

문제가 있을 때 해당 패키지만 clean build하려면:

``` bash
cd ~/ros2_icp_ekf_localization

rm -rf build/robot_description
rm -rf install/robot_description

colcon build \
  --symlink-install \
  --packages-select robot_description

source install/setup.bash
```

------------------------------------------------------------------------

## 4. RViz에서 AMR URDF 실행

작성한 `display.launch.py`를 실행합니다.

``` bash
ros2 launch robot_description display.launch.py
```

정상 실행 시 다음 항목을 확인합니다.

-   RViz2 실행
-   AMR body 표시
-   좌/우 wheel 표시
-   LiDAR 형상 표시
-   caster 형상 표시
-   TF tree 표시

------------------------------------------------------------------------

## 5. launch 파일 확인 및 디버깅

`display.launch.py` 내용을 줄 번호와 함께 확인합니다.

``` bash
cat -n \
~/ros2_icp_ekf_localization/src/robot_description/launch/display.launch.py
```

Python 문법 오류를 검사합니다.

``` bash
python3 -m py_compile \
~/ros2_icp_ekf_localization/src/robot_description/launch/display.launch.py
```

정상일 경우 아무 메시지도 출력되지 않습니다.

`generate_launch_description()` 함수가 실제로 로딩되는지 확인할 수도
있습니다.

``` bash
python3 -c "
import importlib.util
p='/home/juheon/ros2_icp_ekf_localization/src/robot_description/launch/display.launch.py'
spec=importlib.util.spec_from_file_location('display_launch', p)
m=importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
print(hasattr(m, 'generate_launch_description'))
"
```

정상 결과:

``` text
True
```

------------------------------------------------------------------------

## 6. src와 install의 launch 파일 비교

수정한 source 파일과 실제 ROS2가 실행하는 install 파일이 같은지
확인합니다.

``` bash
diff \
~/ros2_icp_ekf_localization/src/robot_description/launch/display.launch.py \
~/ros2_icp_ekf_localization/install/robot_description/share/robot_description/launch/display.launch.py
```

아무 출력이 없다면 두 파일이 동일합니다.

------------------------------------------------------------------------

## 7. Robot Description 확인

`robot_description` topic이 존재하는지 확인합니다.

``` bash
ros2 topic list | grep robot_description
```

Robot Description XML을 확인합니다.

``` bash
ros2 topic echo /robot_description --once
```

`robot_state_publisher` parameter에 URDF가 들어갔는지 확인합니다.

``` bash
ros2 param get /robot_state_publisher robot_description
```

------------------------------------------------------------------------

## 8. 실행 중인 ROS2 Node 확인

``` bash
ros2 node list
```

RViz URDF 확인 단계에서는 대표적으로 다음 노드들을 확인합니다.

``` text
/robot_state_publisher
/joint_state_publisher
/rviz2
```

------------------------------------------------------------------------

## 9. Joint State 확인

``` bash
ros2 topic echo /joint_states --once
```

구동 바퀴를 정의한 경우 다음 joint 이름이 나타나는지 확인합니다.

``` text
left_wheel_joint
right_wheel_joint
```

------------------------------------------------------------------------

## 10. Gazebo / ROS-Gazebo 패키지 설치

패키지 목록을 업데이트합니다.

``` bash
sudo apt update
```

ROS2 Humble용 ROS-Gazebo integration을 설치합니다.

``` bash
sudo apt install ros-humble-ros-gz
```

설치 후 ROS2 환경을 다시 불러옵니다.

``` bash
source /opt/ros/humble/setup.bash
```

Gazebo 버전을 확인합니다.

``` bash
ign gazebo --version
```

또는:

``` bash
ign gazebo --versions
```

------------------------------------------------------------------------

## 11. Gazebo 단독 실행 확인

``` bash
ign gazebo
```

Gazebo GUI가 정상적으로 실행되는지 확인합니다.

------------------------------------------------------------------------

## 12. robot_simulation 패키지 생성

Workspace의 `src` 디렉터리로 이동합니다.

``` bash
cd ~/ros2_icp_ekf_localization/src
```

새 simulation 패키지를 생성합니다.

``` bash
ros2 pkg create robot_simulation \
  --build-type ament_cmake
```

필요한 디렉터리를 만듭니다.

``` bash
cd robot_simulation

mkdir -p launch
mkdir -p worlds
mkdir -p config
```

------------------------------------------------------------------------

## 13. Gazebo World 파일 편집

``` bash
nano \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/empty_world.sdf
```

파일 내용을 확인할 때:

``` bash
cat -n \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/empty_world.sdf
```

------------------------------------------------------------------------

## 14. SDF XML 문법 검사

`xmllint`가 없다면 설치합니다.

``` bash
sudo apt install libxml2-utils
```

SDF 파일의 XML 문법을 검사합니다.

``` bash
xmllint --noout \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/empty_world.sdf
```

정상일 경우 아무 출력도 없습니다.

------------------------------------------------------------------------

## 15. empty_world.sdf 파일 크기 확인

`XML_ERROR_EMPTY_DOCUMENT` 문제가 발생했을 때 사용한 명령입니다.

``` bash
wc -c \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/empty_world.sdf \
~/ros2_icp_ekf_localization/install/robot_simulation/share/robot_simulation/worlds/empty_world.sdf
```

`0` byte가 표시된다면 해당 파일이 비어 있는 상태입니다.

좀 더 자세히 확인하려면:

``` bash
ls -l \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/empty_world.sdf \
~/ros2_icp_ekf_localization/install/robot_simulation/share/robot_simulation/worlds/empty_world.sdf
```

------------------------------------------------------------------------

## 16. 원본 SDF를 직접 Gazebo에서 테스트

빌드/install 문제와 SDF 자체의 문제를 분리하기 위해 `src`의 원본 파일을
직접 실행할 수 있습니다.

``` bash
ign gazebo \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/empty_world.sdf
```

이 명령이 성공하면 SDF 자체는 정상이며, install/build 쪽을 확인하면
됩니다.

------------------------------------------------------------------------

## 17. robot_simulation 빌드

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select robot_simulation
```

빌드 후:

``` bash
source install/setup.bash
```

------------------------------------------------------------------------

## 18. robot_simulation Clean Build

SDF 또는 설치 파일이 제대로 갱신되지 않는 경우:

``` bash
cd ~/ros2_icp_ekf_localization

rm -rf build/robot_simulation
rm -rf install/robot_simulation

colcon build \
  --symlink-install \
  --packages-select robot_simulation

source install/setup.bash
```

------------------------------------------------------------------------

## 19. install 경로의 SDF 확인

``` bash
cat -n \
~/ros2_icp_ekf_localization/install/robot_simulation/share/robot_simulation/worlds/empty_world.sdf
```

원본과 install 파일이 모두 정상인지 확인합니다.

------------------------------------------------------------------------

## 20. install 경로의 Gazebo World 직접 실행

``` bash
ign gazebo \
~/ros2_icp_ekf_localization/install/robot_simulation/share/robot_simulation/worlds/empty_world.sdf
```

이 단계에서 확인할 내용:

-   Gazebo GUI 정상 실행
-   Ground plane 표시
-   조명 정상
-   Physics 정상 동작

------------------------------------------------------------------------

## 21. robot_description + robot_simulation 동시 빌드

URDF와 simulation 설정을 함께 수정한 경우:

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select \
  robot_description \
  robot_simulation
```

빌드 후:

``` bash
source install/setup.bash
```

------------------------------------------------------------------------

## 22. Gazebo Simulation Launch

AMR을 Gazebo World에 Spawn하는 launch 파일을 실행합니다.

``` bash
ros2 launch robot_simulation simulation.launch.py
```

현재 완료된 목표:

``` text
empty_world.sdf
       ↓
Gazebo Fortress
       ↓
robot_state_publisher
       ↓
AMR Spawn
       ↓
Gravity / Collision
       ↓
Ground Contact
       ↓
Stable Robot
```

------------------------------------------------------------------------

## 23. Caster 추가 후 재빌드

앞/뒤 sphere caster 및 friction 관련 Xacro를 수정했다면:

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select robot_description robot_simulation

source install/setup.bash
```

다시 simulation을 실행합니다.

``` bash
ros2 launch robot_simulation simulation.launch.py
```

확인할 내용:

-   좌/우 drive wheel이 바닥에 접촉
-   front/rear caster가 바닥을 지지
-   차체가 한쪽으로 기울지 않음
-   Spawn 후 로봇이 안정적으로 정지
-   caster가 과도한 마찰을 발생시키지 않음

------------------------------------------------------------------------

# 24. 자주 사용하는 개발 루틴

앞으로 가장 자주 사용할 패턴입니다.

### URDF/Xacro 수정 후

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select robot_description

source install/setup.bash

ros2 launch robot_description display.launch.py
```

### Gazebo 관련 파일 수정 후

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select robot_description robot_simulation

source install/setup.bash

ros2 launch robot_simulation simulation.launch.py
```

### 이상 동작 시 Clean Build

``` bash
cd ~/ros2_icp_ekf_localization

rm -rf build/robot_description
rm -rf install/robot_description
rm -rf build/robot_simulation
rm -rf install/robot_simulation

colcon build \
  --symlink-install \
  --packages-select robot_description robot_simulation

source install/setup.bash
```

------------------------------------------------------------------------

# 25. Git 상태 확인

프로젝트 루트로 이동합니다.

``` bash
cd ~/ros2_icp_ekf_localization
```

변경된 파일을 확인합니다.

``` bash
git status
```

변경 내용을 확인합니다.

``` bash
git diff
```

------------------------------------------------------------------------

# 26. Git Staging

이번 단계에서 변경한 파일을 명시적으로 추가하는 방식을 권장합니다.

예:

``` bash
git add src/robot_description
git add src/robot_simulation
git add README.md
git add docs/
```

Staging된 내용을 확인합니다.

``` bash
git status
```

실제로 commit될 변경사항을 확인합니다.

``` bash
git diff --cached
```

------------------------------------------------------------------------

# 27. 현재 단계 Git Commit

현재까지의 AMR URDF + Gazebo Simulation 작업을 하나의 milestone으로
commit한다면:

``` bash
git commit -m "feat: add AMR URDF and Gazebo simulation"
```

commit history 확인:

``` bash
git log --oneline
```

GitHub로 push:

``` bash
git push
```

최초 push에서 upstream이 설정되지 않았다면:

``` bash
git push -u origin main
```

------------------------------------------------------------------------

# 28. 현재 프로젝트 진행 상태

``` text
[✓] Ubuntu 22.04 / ROS2 Humble
[✓] ROS2 Workspace
[✓] robot_description package
[✓] AMR URDF/Xacro
[✓] base_link / base_footprint
[✓] Left / Right Drive Wheel
[✓] LiDAR Link
[✓] IMU Link
[✓] Front / Rear Sphere Caster
[✓] RViz RobotModel
[✓] TF Validation
[✓] Gazebo Fortress
[✓] empty_world.sdf
[✓] Robot Spawn
[✓] Gravity / Collision Test
[✓] Stable Ground Contact

[✓] Differential Drive
[✓] /cmd_vel (Gazebo)
[✓] Wheel Odometry (Gazebo)
[✓] /odom
[✓] 2D LiDAR Simulation
[✓] /scan
[ ] IMU Simulation
[ ] /imu
[ ] Warehouse World
[ ] Mapping
[ ] ICP Localization
[ ] EKF Sensor Fusion
[ ] Ground Truth Evaluation
```

------------------------------------------------------------------------

# 29. 다음 단계

Differential Drive Gazebo 직접 구동 검증까지 완료했습니다.

목표:

``` text
/cmd_vel
    ↓
Differential Drive
    ↓
Left / Right Wheel
    ↓
AMR Motion
    ↓
/odom
```

완료 조건:

1.  `/cmd_vel`로 직진 가능
2.  `/cmd_vel`로 후진 가능
3.  `/cmd_vel`로 좌/우 회전 가능
4.  제자리 회전 가능
5.  `/odom` 생성 확인
6.  Gazebo에서 안정적인 wheel/caster contact 확인
7.  RViz에서 TF/Odometry 확인
8.  실행 결과 이미지 및 GIF 저장
9.  README/TIMELINE 업데이트
10. Git commit 및 push


------------------------------------------------------------------------

# 30. Differential Drive Gazebo Topic 확인

Gazebo simulation을 실행합니다.

``` bash
cd ~/ros2_icp_ekf_localization

source install/setup.bash

ros2 launch robot_simulation simulation.launch.py
```

Gazebo Transport topic을 확인합니다.

``` bash
ign topic -l | grep -E "cmd_vel|odom|odometry|icp_ekf_amr"
```

확인된 topic:

``` text
/model/icp_ekf_amr/odometry
/model/icp_ekf_amr/tf
```

------------------------------------------------------------------------

# 31. Differential Drive 직접 구동 테스트

전진:

``` bash
ign topic -t /cmd_vel -m ignition.msgs.Twist -p "linear: {x: 0.2}, angular: {z: 0.0}"
```

후진:

``` bash
ign topic -t /cmd_vel -m ignition.msgs.Twist -p "linear: {x: -0.2}, angular: {z: 0.0}"
```

반시계 방향 회전:

``` bash
ign topic -t /cmd_vel -m ignition.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.5}"
```

시계 방향 회전:

``` bash
ign topic -t /cmd_vel -m ignition.msgs.Twist -p "linear: {x: 0.0}, angular: {z: -0.5}"
```

정상 동작 기준:

``` text
linear.x > 0  → Forward (+X / LiDAR 방향)
linear.x < 0  → Backward
angular.z > 0 → Counter-clockwise
angular.z < 0 → Clockwise
```

------------------------------------------------------------------------

# 32. Gazebo Odometry 확인

``` bash
ign topic -e -t /model/icp_ekf_amr/odometry
```

로봇이 움직일 때 position 및 orientation 값이 변화하는지 확인합니다.

------------------------------------------------------------------------

# 33. Wheel Joint Axis 방향 수정

초기 테스트에서 다음 문제가 발생했습니다.

``` text
linear.x > 0  → 로봇이 뒤로 이동
angular.z > 0 → 로봇이 오른쪽으로 회전
```

기존 wheel joint axis:

``` xml
<axis xyz="0 0 1"/>
```

수정:

``` xml
<axis xyz="0 0 -1"/>
```

`left_wheel_joint`와 `right_wheel_joint` 모두 수정했습니다.

수정 후 다시 빌드합니다.

``` bash
cd ~/ros2_icp_ekf_localization

colcon build   --symlink-install   --packages-select robot_description robot_simulation

source install/setup.bash
```

Gazebo를 다시 실행합니다.

``` bash
ros2 launch robot_simulation simulation.launch.py
```

수정 후 확인 결과:

``` text
[✓] Forward
[✓] Backward
[✓] Counter-clockwise Rotation
[✓] Clockwise Rotation
[✓] Gazebo Odometry
```

------------------------------------------------------------------------

# 34. 다음 단계

다음 개발 milestone은 **ROS2 ↔ Gazebo Bridge**입니다.

목표:

``` text
ROS2 /cmd_vel
    ↓
ros_gz_bridge
    ↓
Gazebo Differential Drive
    ↓
AMR Motion
    ↓
Gazebo Odometry
    ↓
ros_gz_bridge
    ↓
ROS2 /odom
```

------------------------------------------------------------------------

# 35. Git Commit

``` bash
cd ~/ros2_icp_ekf_localization

git status

git diff
```

변경 파일을 staging 합니다.

``` bash
git add src/robot_description/urdf/amr.urdf.xacro
git add README.md
git add COMMANDS.md
git add TIMELINE.md
```

Staging 결과를 확인합니다.

``` bash
git status

git diff --cached
```

Commit:

``` bash
git commit -m "feat: add differential drive and validate robot motion"
```

Push:

``` bash
git push origin main
```


------------------------------------------------------------------------

# 36. ROS2 ↔ Gazebo Bridge 자동 실행

`simulation.launch.py`에 `ros_gz_bridge`를 추가하여 Gazebo simulation과 함께
bridge가 자동으로 실행되도록 구성했습니다.

Simulation 실행:

``` bash
cd ~/ros2_icp_ekf_localization

source /opt/ros/humble/setup.bash

source install/setup.bash

ros2 launch robot_simulation simulation.launch.py
```

Launch 로그에서 `parameter_bridge` process가 자동으로 실행되는지 확인합니다.

예:

``` text
[INFO] [parameter_bridge-4]: process started with pid [...]
```

------------------------------------------------------------------------

# 37. ROS2 Topic 확인

새 터미널에서:

``` bash
source /opt/ros/humble/setup.bash

source ~/ros2_icp_ekf_localization/install/setup.bash
```

ROS2 topic을 확인합니다.

``` bash
ros2 topic list
```

확인 대상:

``` text
/cmd_vel
/odom
```

상세 정보 확인:

``` bash
ros2 topic info /cmd_vel

ros2 topic info /odom
```

------------------------------------------------------------------------

# 38. ROS2 /cmd_vel로 AMR 구동 확인

이 단계부터는 `ign topic`이 아니라 ROS2 topic을 사용합니다.

전진:

``` bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 10
```

반시계 방향 회전:

``` bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -r 10
```

시계 방향 회전:

``` bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}" -r 10
```

정상 동작 기준:

``` text
linear.x > 0  → Forward (+X / LiDAR 방향)
linear.x < 0  → Backward
angular.z > 0 → Counter-clockwise
angular.z < 0 → Clockwise
```

------------------------------------------------------------------------

# 39. ROS2 /odom 확인

새 터미널에서:

``` bash
ros2 topic echo /odom
```

로봇을 움직였을 때 다음 값이 변화하는지 확인합니다.

``` text
pose.pose.position.x
pose.pose.position.y
pose.pose.orientation
twist.twist.linear
twist.twist.angular
```

ROS2 `/cmd_vel` 명령으로 AMR이 움직이고 `/odom` 값이 변화하면
ROS2 ↔ Gazebo Bridge 연결은 정상입니다.

------------------------------------------------------------------------

# 40. Bridge Launch 통합 완료 상태

``` text
[✓] ros_gz_bridge
[✓] Bridge automatic launch
[✓] ROS2 /cmd_vel
[✓] ROS2 → Gazebo velocity command
[✓] Gazebo Differential Drive
[✓] Gazebo → ROS2 odometry
[✓] ROS2 /odom
[✓] AMR motion validation
```

현재 데이터 흐름:

``` text
ROS2 /cmd_vel
    ↓
ros_gz_bridge
    ↓
Gazebo Differential Drive
    ↓
AMR Motion
    ↓
Gazebo Odometry
    ↓
ros_gz_bridge
    ↓
ROS2 /odom
```

------------------------------------------------------------------------

# 41. Bridge 단계 Git Commit

변경 사항 확인:

``` bash
cd ~/ros2_icp_ekf_localization

git status

git diff
```

이번 단계에서 수정한 파일을 staging 합니다.

``` bash
git add src/robot_simulation/launch/simulation.launch.py
git add src/robot_simulation/config/
git add README.md
git add docs/COMMANDS.md
git add docs/TIMELINE.md
```

Bridge 동작 확인 이미지 또는 GIF를 추가했다면:

``` bash
git add docs/images/
```

Commit 전 확인:

``` bash
git status

git diff --cached
```

권장 commit:

``` bash
git commit -m "feat: integrate ROS2 Gazebo bridge for cmd_vel and odometry"
```

Push:

``` bash
git push origin main
```

------------------------------------------------------------------------

# 42. 다음 단계

다음 개발 milestone은 **2D LiDAR Simulation**입니다.

목표:

``` text
Gazebo LiDAR
    ↓
Gazebo LaserScan
    ↓
ros_gz_bridge
    ↓
ROS2 /scan
    ↓
RViz LaserScan
```

------------------------------------------------------------------------

# 43. Gazebo Wheel Joint State 확인

``` bash
ign topic -l | grep -E "joint|joint_state"
```

``` bash
ign topic -i -t /model/icp_ekf_amr/joint_state
```

``` bash
ign topic -e -t /model/icp_ekf_amr/joint_state
```

확인 대상:

``` text
left_wheel_joint
right_wheel_joint
```

------------------------------------------------------------------------

# 44. ROS2 /joint_states 및 Wheel TF 확인

Gazebo joint state가 `ros_gz_bridge`를 통해 ROS2로 전달되는지 확인합니다.

``` bash
ros2 topic info /joint_states --verbose
ros2 topic echo /joint_states --once
```

로봇을 움직이는 동안 wheel position 값이 변화하는지 확인합니다.

``` bash
ros2 topic echo /joint_states
```

Wheel TF 확인:

``` bash
ros2 run tf2_ros tf2_echo base_link left_wheel_link
ros2 run tf2_ros tf2_echo base_link right_wheel_link
```

별도의 `joint_state_publisher`를 실행하지 않아도 wheel TF와 RViz2의 좌/우 wheel이 표시되면 정상입니다.

------------------------------------------------------------------------

# 45. Gazebo / ROS2 LiDAR Topic 확인

``` bash
ign topic -l | grep -E "scan|lidar"
ign topic -e -t /scan
```

ROS2 bridge 확인:

``` bash
ros2 topic list | grep scan
ros2 topic type /scan
ros2 topic echo /scan --once | head -20
```

정상 message type:

``` text
sensor_msgs/msg/LaserScan
```

------------------------------------------------------------------------

# 46. LiDAR Publish Frequency 확인

``` bash
ros2 topic hz /scan
```

현재 LiDAR 설정의 목표 update rate는 약 `10 Hz`입니다.

------------------------------------------------------------------------

# 47. LiDAR TF / Frame 확인

``` bash
ros2 run tf2_ros tf2_echo base_link laser_link
```

``` bash
ros2 topic echo /scan --once | head -10
```

초기 테스트에서는 Gazebo sensor frame과 ROS2 TF frame이 일치하지 않아 RViz2에서 다음 로그가 반복되었습니다.

``` text
Message Filter dropping message
discarding message because the queue is full
```

LiDAR sensor frame을 ROS2 TF tree의 `laser_link`와 일치시킨 후 해당 로그가 사라졌습니다.

------------------------------------------------------------------------

# 48. RViz2 LaserScan 시각화

``` bash
rviz2
```

LaserScan display 확인:

``` text
Topic: /scan
Status: Ok
```

Gazebo world의 static obstacle이 RViz2에서 LaserScan point로 표시되는지 확인합니다.

다음 RViz2 시작 로그는 정상 렌더링과 LaserScan 표시가 된다면 LiDAR 오류가 아닙니다.

``` text
Warning: Ignoring XDG_SESSION_TYPE=wayland on Gnome.
Stereo is NOT SUPPORTED
OpenGl version: 4.6 (GLSL 4.6)
```

------------------------------------------------------------------------

# 49. LiDAR 단계 완료 상태

``` text
[✓] Gazebo GPU LiDAR
[✓] 360° scan / 360 samples
[✓] 0.10 m ~ 10.0 m range
[✓] 10 Hz update rate
[✓] Gazebo /scan → ros_gz_bridge → ROS2 /scan
[✓] LiDAR TF alignment
[✓] RViz2 LaserScan
[✓] Static obstacle detection
[✓] RViz Message Filter frame issue resolved
```

------------------------------------------------------------------------

# 50. 다음 단계

다음 개발 milestone은 **IMU Simulation**입니다.

``` text
Gazebo IMU
    ↓
Gazebo IMU Message
    ↓
ros_gz_bridge
    ↓
ROS2 /imu
    ↓
Localization Pipeline
```

------------------------------------------------------------------------

# 51. Gazebo IMU System 및 Sensor 확인

`empty_world.sdf`에 Gazebo IMU system을 추가하고 `amr.urdf.xacro`의 `imu_link`에 IMU sensor를 연결한 후 빌드합니다.

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select robot_description robot_simulation

source install/setup.bash
```

Simulation 실행:

``` bash
ros2 launch robot_simulation simulation.launch.py
```

Gazebo IMU topic 확인:

``` bash
ign topic -l | grep imu
```

``` bash
ign topic -e -t /imu
```

확인 대상:

``` text
orientation
angular_velocity
linear_acceleration
```

------------------------------------------------------------------------

# 52. ROS2 /imu Bridge 확인

`bridge.yaml`에 Gazebo `/imu` → ROS2 `/imu` bridge를 추가한 후 확인합니다.

``` bash
ros2 topic list | grep imu
```

``` bash
ros2 topic type /imu
```

정상 message type:

``` text
sensor_msgs/msg/Imu
```

IMU data 확인:

``` bash
ros2 topic echo /imu --once
```

Publish frequency 확인:

``` bash
ros2 topic hz /imu
```

현재 설정의 목표 update rate는 약 `50 Hz`입니다.

------------------------------------------------------------------------

# 53. IMU Frame 및 Motion Validation

IMU message frame 확인:

``` bash
ros2 topic echo /imu --once | head -15
```

정상 frame:

``` text
imu_link
```

TF 확인:

``` bash
ros2 run tf2_ros tf2_echo base_link imu_link
```

회전 명령:

``` bash
ros2 topic pub \
/cmd_vel \
geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}" \
-r 10
```

회전 중 `/imu`의 `angular_velocity.z` 값이 변화하는지 확인합니다.

초기에는 Gazebo scoped sensor frame이 `/imu`의 `frame_id`로 사용되었으며, ROS2 TF tree의 `imu_link`와 일치하도록 sensor frame을 수정했습니다.

------------------------------------------------------------------------

# 54. Localization Benchmark World 생성 및 검증

평가용 world:

``` text
src/robot_simulation/worlds/localization_world.sdf
```

XML 문법 확인:

``` bash
xmllint --noout \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/localization_world.sdf
```

정상일 경우 아무 출력도 없습니다.

World 단독 실행:

``` bash
ign gazebo \
~/ros2_icp_ekf_localization/src/robot_simulation/worlds/localization_world.sdf
```

확인할 내용:

``` text
[✓] Outer walls
[✓] Different-sized landmarks
[✓] Rotated obstacle
[✓] Central rectangular structure
[✓] L-shaped feature
[✓] Asymmetric geometry
```

------------------------------------------------------------------------

# 55. World 및 Initial Pose Launch Argument 확인

Launch argument 확인:

``` bash
ros2 launch robot_simulation simulation.launch.py --show-args
```

확인 대상:

``` text
world
x
y
z
yaw
```

기존 sensor test world 실행:

``` bash
ros2 launch robot_simulation simulation.launch.py
```

Benchmark world 실행:

``` bash
ros2 launch robot_simulation simulation.launch.py \
world:=localization_world.sdf \
x:=0.0 \
y:=-3.8 \
yaw:=1.5708
```

이 설정을 향후 localization 평가의 고정 초기 조건으로 사용합니다.

------------------------------------------------------------------------

# 56. Odometry TF 확인

ROS2 `/odom` message의 frame 설정을 확인합니다.

``` bash
ros2 topic echo /odom --once | head -20
```

목표:

``` text
header.frame_id: odom
child_frame_id: base_footprint
```

TF 확인:

``` bash
ros2 run tf2_ros tf2_echo odom base_footprint
```

로봇을 움직였을 때 Translation / Rotation 값이 변화하는지 확인합니다.

현재 TF chain:

``` text
odom
  ↓
base_footprint
  ↓
base_link
  ↓
laser_link
```

------------------------------------------------------------------------

# 57. SLAM Toolbox 설치 및 Configuration

설치 여부 확인:

``` bash
ros2 pkg list | grep slam_toolbox
```

설치되어 있지 않다면:

``` bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

기본 설정 파일 복사:

``` bash
cp \
/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml \
~/ros2_icp_ekf_localization/src/robot_simulation/config/slam_toolbox.yaml
```

주요 설정:

``` text
odom_frame: odom
map_frame: map
base_frame: base_footprint
scan_topic: /scan
mode: mapping
min_laser_range: 0.1
max_laser_range: 10.0
```

------------------------------------------------------------------------

# 58. SLAM Toolbox Mapping 실행

먼저 benchmark simulation을 실행합니다.

``` bash
ros2 launch robot_simulation simulation.launch.py \
world:=localization_world.sdf \
x:=0.0 \
y:=-3.8 \
yaw:=1.5708
```

새 터미널에서 SLAM Toolbox 실행:

``` bash
ros2 launch slam_toolbox online_async_launch.py \
use_sim_time:=true \
slam_params_file:=$HOME/ros2_icp_ekf_localization/src/robot_simulation/config/slam_toolbox.yaml
```

Map topic 확인:

``` bash
ros2 topic list | grep map
```

Map TF 확인:

``` bash
ros2 run tf2_ros tf2_echo map odom
```

------------------------------------------------------------------------

# 59. RViz2 Mapping Visualization

``` bash
rviz2
```

RViz2 설정:

``` text
Fixed Frame: map
Map Topic: /map
LaserScan Topic: /scan
```

Mapping 중 전체 TF chain:

``` text
map
  ↓
odom
  ↓
base_footprint
  ↓
base_link
  ↓
laser_link
```

------------------------------------------------------------------------

# 60. Teleoperation을 이용한 Mapping 주행

Teleop package 실행:

``` bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

설치되어 있지 않다면:

``` bash
sudo apt install ros-humble-teleop-twist-keyboard
```

급격한 이동과 회전을 피하고 benchmark world의 벽과 landmark가 충분히 관측되도록 주행합니다.

------------------------------------------------------------------------

# 61. Occupancy Grid Map 저장

Map 저장 디렉터리 생성:

``` bash
cd ~/ros2_icp_ekf_localization
mkdir -p maps
```

SLAM Toolbox가 실행 중인 상태에서 map을 저장합니다.

``` bash
ros2 run nav2_map_server map_saver_cli \
-f ~/ros2_icp_ekf_localization/maps/localization_map
```

생성 파일:

``` text
maps/
├── localization_map.pgm
└── localization_map.yaml
```

저장 결과 확인:

``` bash
ls -lh ~/ros2_icp_ekf_localization/maps/
```

``` bash
cat ~/ros2_icp_ekf_localization/maps/localization_map.yaml
```

------------------------------------------------------------------------

# 62. Saved Map Reload - Map Server 실행

SLAM Toolbox를 종료한 후 저장된 map을 Map Server에서 다시 불러옵니다.

``` bash
ros2 run nav2_map_server map_server \
--ros-args \
-p yaml_filename:=$HOME/ros2_icp_ekf_localization/maps/localization_map.yaml \
-p use_sim_time:=true
```

`map_server`는 Lifecycle Node이므로 별도의 configure / activate 과정이 필요합니다.

------------------------------------------------------------------------

# 63. Map Server Lifecycle Control

현재 상태 확인:

``` bash
ros2 lifecycle get /map_server
```

Configure:

``` bash
ros2 lifecycle set /map_server configure
```

Activate:

``` bash
ros2 lifecycle set /map_server activate
```

최종 상태 확인:

``` bash
ros2 lifecycle get /map_server
```

정상 상태:

``` text
active [3]
```

------------------------------------------------------------------------

# 64. /map QoS 확인 및 OccupancyGrid 수신

Map Server에서 `/map` topic은 durable map data를 제공하므로 subscriber QoS를 명시하여 확인합니다.

``` bash
ros2 topic info /map --verbose
```

``` bash
ros2 topic echo /map \
--qos-reliability reliable \
--qos-durability transient_local \
--once
```

정상 message type:

``` text
nav_msgs/msg/OccupancyGrid
```

현재 생성된 map 정보:

``` text
Resolution: 0.05 m/cell
Width: 199 cells
Height: 198 cells
```

초기에는 `/map` topic이 존재했지만 기본 `ros2 topic echo`로 데이터가 출력되지 않았습니다. `Reliable + Transient Local` QoS를 명시한 후 저장된 OccupancyGrid를 정상적으로 수신했습니다.

------------------------------------------------------------------------

# 65. RViz2 Saved Map Reload Validation

RViz2 실행:

``` bash
rviz2
```

Map display 설정:

``` text
Topic: /map
Reliability Policy: Reliable
Durability Policy: Transient Local
```

검증 결과:

``` text
Map Status: Ok
```

Map Server는 저장된 `/map`을 publish하지만 `map -> odom` TF를 생성하지 않습니다.

따라서 SLAM Toolbox 또는 localization node가 실행되지 않은 상태에서는 RViz2의 Global Status에서 다음 메시지가 나타날 수 있습니다.

``` text
Fixed Frame [map] does not exist
```

이 상태에서도 Map display가 `Status: Ok`이고 저장된 Occupancy Grid가 정상적으로 표시된다면 saved map reload 자체는 정상입니다.

RViz2에서 다음 GLSL message가 나타날 수 있습니다.

``` text
active samplers with a different type refer to the same texture image unit
```

Map이 정상적으로 렌더링된다면 현재 mapping / map reload 기능 검증에는 영향을 주지 않습니다.

------------------------------------------------------------------------

# 66. Mapping 단계 완료 상태

``` text
[✓] Gazebo IMU Simulation
[✓] ROS2 /imu
[✓] IMU frame alignment
[✓] localization_world.sdf
[✓] World launch argument
[✓] Fixed initial pose argument
[✓] odom -> base_footprint TF
[✓] SLAM Toolbox Mapping
[✓] map -> odom TF during mapping
[✓] ROS2 /map OccupancyGrid
[✓] localization_map.pgm
[✓] localization_map.yaml
[✓] Map Server reload
[✓] Reliable + Transient Local QoS validation
[✓] RViz2 saved map visualization
```

------------------------------------------------------------------------

# 67. 다음 단계

다음 개발 milestone은 **Custom ICP Localization**입니다.

``` text
Saved Occupancy Map
        ↓
Reference Point Cloud
        +
Current LiDAR /scan
        ↓
Correspondence Search
        ↓
Rigid Transform Estimation
        ↓
Iterative Optimization
        ↓
ICP Pose [x, y, yaw]
```

------------------------------------------------------------------------

# 68. Custom ICP Localization Package 생성

프로젝트 `src` 디렉터리에서 C++ package를 생성합니다.

``` bash
cd ~/ros2_icp_ekf_localization/src

ros2 pkg create icp_localization \
  --build-type ament_cmake \
  --dependencies rclcpp sensor_msgs
```

Occupancy Grid를 사용하기 위해 `package.xml`과 `CMakeLists.txt`에 `nav_msgs` dependency를 추가합니다.

`package.xml`:

``` xml
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
```

`CMakeLists.txt`:

``` cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
```

------------------------------------------------------------------------

# 69. ICP Package Build

``` bash
cd ~/ros2_icp_ekf_localization

colcon build \
  --symlink-install \
  --packages-select icp_localization

source install/setup.bash
```

정상 build 결과 예:

``` text
Finished <<< icp_localization
Summary: 1 package finished
```

------------------------------------------------------------------------

# 70. LaserScan -> 2D Point 변환 Node 실행

먼저 benchmark simulation을 실행합니다.

``` bash
ros2 launch robot_simulation simulation.launch.py \
world:=localization_world.sdf \
x:=0.0 \
y:=-3.8 \
yaw:=1.5708
```

새 터미널에서 ICP node를 실행합니다.

``` bash
source /opt/ros/humble/setup.bash
source ~/ros2_icp_ekf_localization/install/setup.bash

ros2 run icp_localization icp_localization_node
```

LaserScan 변환 결과 확인:

``` text
Converted LaserScan to XXX 2D points
```

변환 과정:

``` text
/scan
  ↓
finite / range filtering
  ↓
angle = angle_min + i * angle_increment
  ↓
x = range * cos(angle)
y = range * sin(angle)
  ↓
2D Point
```

------------------------------------------------------------------------

# 71. /icp_scan_points PointCloud2 확인

Topic 확인:

``` bash
ros2 topic list | grep icp
```

Message type 확인:

``` bash
ros2 topic type /icp_scan_points
```

정상 결과:

``` text
sensor_msgs/msg/PointCloud2
```

Publish rate 확인:

``` bash
ros2 topic hz /icp_scan_points
```

LiDAR update rate와 동일하게 약 `10 Hz`인지 확인합니다.

RViz2 실행:

``` bash
rviz2
```

검증 설정:

``` text
Fixed Frame: laser_link
LaserScan Topic: /scan
PointCloud2 Topic: /icp_scan_points
```

정상 결과:

``` text
/scan과 /icp_scan_points가 동일한 장애물 윤곽에서 겹쳐 표시됨
```

검증 이미지:

``` text
docs/images/10_laserscan_pointcloud_overlap_rviz.gif
```

------------------------------------------------------------------------

# 72. ICP Reference Map 실행 환경

저장된 Occupancy Grid를 사용하기 위해 Map Server를 실행합니다.

``` bash
ros2 run nav2_map_server map_server \
--ros-args \
-p yaml_filename:=$HOME/ros2_icp_ekf_localization/maps/localization_map.yaml \
-p use_sim_time:=true
```

Lifecycle 활성화:

``` bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

상태 확인:

``` bash
ros2 lifecycle get /map_server
```

정상 상태:

``` text
active [3]
```

------------------------------------------------------------------------

# 73. ICP Node의 /map 수신 확인

ICP node의 `/map` subscriber는 Map Server QoS와 맞추기 위해 다음 profile을 사용합니다.

``` text
History: Keep Last
Depth: 1
Reliability: Reliable
Durability: Transient Local
```

ICP node 실행:

``` bash
ros2 run icp_localization icp_localization_node
```

정상 출력 예:

``` text
Map received: width=199, height=198, resolution=0.050
```

Occupancy Grid 정보:

``` text
Width: 199 cells
Height: 198 cells
Resolution: 0.05 m/cell
```

------------------------------------------------------------------------

# 74. OccupancyGrid -> Reference PointCloud 변환 확인

Reference point 생성 로그 확인:

``` text
Converted OccupancyGrid to XXXX reference points
```

현재 occupied-cell 기준:

``` text
occupancy >= 65
```

Grid index 변환:

``` text
column = index % width
row    = index / width
```

Map metric coordinate 변환:

``` text
x = origin_x + (column + 0.5) * resolution
y = origin_y + (row + 0.5) * resolution
```

`+0.5`는 각 grid cell의 모서리가 아니라 중심점을 reference point로 사용하기 위한 값입니다.

------------------------------------------------------------------------

# 75. /icp_map_points PointCloud2 확인

Topic 확인:

``` bash
ros2 topic list | grep icp
```

현재 ICP 입력 topic:

``` text
/icp_scan_points
/icp_map_points
```

Message type 확인:

``` bash
ros2 topic type /icp_map_points
```

정상 결과:

``` text
sensor_msgs/msg/PointCloud2
```

Map point cloud는 정적인 reference data이므로 `Reliable + Transient Local` QoS를 사용합니다.

Message 확인:

``` bash
ros2 topic echo /icp_map_points \
--qos-reliability reliable \
--qos-durability transient_local \
--once
```

확인 항목:

``` text
header.frame_id: map
width > 0
```

------------------------------------------------------------------------

# 76. RViz2 OccupancyGrid / Reference PointCloud 검증

RViz2 실행:

``` bash
rviz2
```

Map display 설정:

``` text
Topic: /map
Reliability: Reliable
Durability: Transient Local
```

PointCloud2 display 설정:

``` text
Topic: /icp_map_points
Reliability: Reliable
Durability: Transient Local
Size: 0.03 ~ 0.05 m
```

Map Server만 실행 중인 경우 `map -> odom` TF가 존재하지 않으므로 시각화 검증이 필요할 때 임시 static TF를 사용할 수 있습니다.

``` bash
ros2 run tf2_ros static_transform_publisher \
--x 0 \
--y 0 \
--z 0 \
--yaw 0 \
--pitch 0 \
--roll 0 \
--frame-id map \
--child-frame-id odom
```

주의:

``` text
이 static transform은 RViz2 입력 데이터 검증용입니다.
실제 ICP localization 결과로 사용하지 않습니다.
```

정상 결과:

``` text
OccupancyGrid의 벽 / 장애물 영역과 /icp_map_points가 동일한 위치에서 겹쳐 표시됨
```

검증 GIF:

``` text
docs/images/11_occupancygrid_reference_pointcloud_rviz.gif
```

------------------------------------------------------------------------

# 77. Custom ICP 현재 진행 상태

``` text
[✓] icp_localization C++ package
[✓] ROS2 /scan subscriber
[✓] Invalid LaserScan range filtering
[✓] LaserScan beam angle calculation
[✓] Polar -> Cartesian conversion
[✓] /icp_scan_points PointCloud2
[✓] /scan / PointCloud2 RViz2 overlap validation
[✓] ROS2 /map OccupancyGrid subscriber
[✓] Reliable + Transient Local map QoS
[✓] Occupied cell extraction
[✓] Grid index -> row / column conversion
[✓] Grid cell -> map coordinate conversion
[✓] /icp_map_points PointCloud2
[✓] OccupancyGrid / Reference PointCloud RViz2 validation

[ ] Scan points -> map frame transformation
[ ] Odometry initial pose integration
[ ] Correspondence search
[ ] Outlier rejection
[ ] 2D rigid transform estimation
[ ] ICP iteration / convergence
[ ] ICP pose publication
```

------------------------------------------------------------------------

# 78. 다음 단계

다음 개발 단계는 **Current Scan Point를 Map Frame으로 변환하는 과정**입니다.

현재 좌표계:

``` text
/icp_scan_points
Frame: laser_link

/icp_map_points
Frame: map
```

두 point set을 ICP correspondence 단계에서 직접 비교하기 전에 동일한 좌표계로 변환해야 합니다.

다음 목표:

``` text
Wheel Odometry / TF
        ↓
Initial Pose Estimate
        ↓
Current Scan Points
laser_link -> map
        ↓
Map-frame Scan Points
        ↓
Reference Map Points와 Correspondence Search
```

