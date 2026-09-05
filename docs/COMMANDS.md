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
[ ] /odom
[ ] 2D LiDAR Simulation
[ ] /scan
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
