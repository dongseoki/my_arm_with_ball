# my_arm_with_ball

UR5 · RG2 기반 ROS 2 Pick & Place 시뮬레이션 프로젝트

A ROS 2 pick-and-place simulation project with a UR5 arm and RG2 gripper.

[한국어](#한국어) · [English](#english)

## 한국어

### 프로젝트 소개

UR5 로봇 팔과 RG2 그리퍼로 물체를 집어 바구니에 옮기는 과정을 ROS 2와 Gazebo에서 구현하는 프로젝트입니다. MoveIt 2의 경로 계획, ros2_control의 궤적 실행, Gazebo의 물리 시뮬레이션을 연결합니다.

현재 자동 작업은 **단일 구형 물체(`apple`)를 대상으로 하는 시뮬레이션 데모**입니다. 실물 로봇 연동이나 비전 기반 물체 인식을 제공하는 프로젝트는 아닙니다.

주요 기능은 다음과 같습니다.

- **자동 Pick & Place:** MoveIt Task Constructor(MTC)로 접근·잡기·들기·운반·놓기·복귀를 구성합니다.
- **잡기와 놓기 확인:** Gazebo의 접촉·부착 상태와 물체 위치를 확인하고, MoveIt Planning Scene의 부착 상태를 동기화합니다. 놓은 뒤에는 바구니 안에서 물체가 안정됐는지 확인합니다.
- **수동 조작:** RViz의 MotionPlanning 패널 또는 터미널 키보드로 관절을 움직일 수 있습니다.
- **단계별 테스트:** 경로 계획, 잡기, 전체 Pick & Place를 나눠 확인할 수 있습니다.

### 시스템 구성

| 구성 요소 | 역할 |
| --- | --- |
| MoveIt 2 / MTC | 충돌을 고려한 경로 계획과 작업 단계 구성 |
| ros2_control | 팔·그리퍼 컨트롤러를 통한 관절 궤적 실행 |
| Gazebo / `GraspSystem` | 물리 시뮬레이션, 접촉 관측, 물체 부착·분리 |
| Planning Scene | 계획에 사용하는 충돌 물체와 부착 물체 상태 관리 |
| RViz | 로봇 상태·계획 장면 시각화와 계획·실행 UI |

Gazebo에서 물체가 실제로 부착됐는지와 Planning Scene에서 부착 물체로 등록됐는지는 별도 상태입니다. 이 프로젝트는 두 상태를 확인하고 동기화하는 흐름을 포함합니다.

| 패키지 | 내용 |
| --- | --- |
| `my_arm_with_ball_description` | UR5/RG2 모델, 메시, SDF/Xacro |
| `my_arm_with_ball_gazebo` | Gazebo 월드와 C++ 시스템 플러그인 |
| `my_arm_with_ball_moveit_config` | MoveIt 설정, 계획 데모, 계획 검증 도구 |
| `my_arm_with_ball_application` | 키보드 조작, 잡기 테스트, MTC Pick & Place |
| `my_arm_with_ball_bringup` | 통합 실행 파일, 컨트롤러·브리지 설정 |

### 사용법

#### 1. 설치 및 빌드

개발 환경은 **Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic**입니다. 아래 명령은 ROS 2와 Gazebo가 설치된 환경을 전제로 합니다.

```bash
git clone https://github.com/dongseoki/my_arm_with_ball.git
cd my_arm_with_ball

source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
sudo apt install python3-colcon-common-extensions python3-rosdep

# 시스템에서 rosdep을 처음 설정하는 경우에만 실행
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

다음 실행 명령은 저장소 루트에서 입력합니다. 새 터미널을 열 때마다 먼저 환경을 불러옵니다.

```bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
source install/setup.bash
```

#### 2. 자동 Pick & Place 실행

전체 동작을 보려면 다음 명령을 실행합니다. Gazebo와 RViz도 함께 시작합니다.

```bash
ros2 launch my_arm_with_ball_bringup pick_and_place_test.launch.py headless:=false rviz:=true
```

- 성공 로그: `STAGE5 PICK AND PLACE PASSED; waiting`
- 실패 로그: `STAGE5 FAILED stage=... reason=...; waiting without retry`
- 성공·실패 후에도 관찰할 수 있도록 노드와 시뮬레이션이 유지됩니다. 자동 재시도는 하지 않습니다.
- GUI 없이 실행하려면 `headless:=true rviz:=false`를 사용합니다.

실행을 끝낼 때는 `Ctrl+C`를 누릅니다. 각 시뮬레이션 실행 예제는 **기존 실행을 종료한 뒤 하나씩** 시작합니다.

#### 3. 시뮬레이션 및 수동 조작

자동 작업 없이 Gazebo, MoveIt, 컨트롤러와 RViz를 시작합니다. RViz의 MotionPlanning 패널에서 계획하고 실행할 수 있습니다.

```bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```

GUI 없이 실행하려면 `headless:=true`를 추가합니다. RViz만 끄려면 `rviz:=false`를 추가합니다.

키보드 조작은 환경을 불러온 **별도의 대화형 터미널**에서 실행합니다.

```bash
ros2 run my_arm_with_ball_application keyboard_joint_teleop
```

| 키 | 동작 |
| --- | --- |
| `1`–`6` | UR5 관절 선택 |
| `a` / `d` | 선택 관절 목표를 −0.05 / +0.05 rad 변경 |
| `o` / `c` | 그리퍼 열기 / 닫기 |
| `r` | 선택한 팔 관절을 0 rad로 이동 |
| `x` | 모든 팔 관절을 0 rad로 이동, 그리퍼 유지 |
| `q` | 키보드 노드 종료 |

키보드 조작은 컨트롤러에 직접 궤적을 보내며 **MoveIt의 충돌 검사를 거치지 않습니다**. 궤적 실행 중 추가 이동 입력은 무시됩니다.

#### 4. 테스트 및 상태 확인

**잡기만 테스트:** 새 시뮬레이션을 시작해 잡기 동작을 검증합니다. 성공 시 `STAGE4 GRASP TEST PASSED`를 출력하고 자동 종료합니다.

```bash
ros2 launch my_arm_with_ball_bringup pick_grasp_test.launch.py headless:=true rviz:=false
```

**경로 계획만 테스트:** Gazebo 없이 MoveIt 데모를 시작하고, 환경을 불러온 별도 터미널에서 검증 도구를 실행합니다. 로봇 궤적은 실행하지 않습니다.

```bash
ros2 launch my_arm_with_ball_moveit_config demo.launch.py
```

```bash
ros2 run my_arm_with_ball_moveit_config validate_planning.py
```

**컨트롤러 상태 확인:** 통합 시뮬레이션 실행 중 별도 터미널에서 확인합니다.

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

**패키지 테스트:** 빌드 후 실행합니다. 위의 시뮬레이션 동작 테스트와는 별도입니다.

```bash
colcon test
colcon test-result --verbose
```

### 모델 출처

UR5/RG2 모델은 [Gazebo Fuel의 `anni/ur5_rg2`](https://fuel.gazebosim.org/1.0/anni/models/ur5_rg2/1)를 기반으로 합니다.

---

## English

### Overview

This project implements picking an object and placing it in a basket with a UR5 arm and RG2 gripper in ROS 2 and Gazebo. It connects MoveIt 2 motion planning, ros2_control trajectory execution, and Gazebo physics simulation.

The current automated task is **a simulation demo for a single spherical object (`apple`)**. The project does not provide physical robot integration or vision-based object recognition.

Key features:

- **Automated pick and place:** MoveIt Task Constructor (MTC) organizes approach, grasp, lift, transport, release, and return motions.
- **Grasp and placement checks:** The application checks Gazebo contact, attachment, and object position, and synchronizes attachment state with the MoveIt Planning Scene. After release, it checks that the object has settled inside the basket.
- **Manual control:** Move joints through the RViz MotionPlanning panel or a terminal keyboard interface.
- **Separate tests:** Check motion planning, grasping, and the full pick-and-place sequence independently.

### System Structure

| Component | Responsibility |
| --- | --- |
| MoveIt 2 / MTC | Collision-aware motion planning and task stages |
| ros2_control | Joint trajectory execution through arm and gripper controllers |
| Gazebo / `GraspSystem` | Physics simulation, contact observation, object attachment and detachment |
| Planning Scene | Collision and attached object state used for planning |
| RViz | Robot and planning scene visualization, with a planning and execution UI |

Physical attachment in Gazebo and attached-object registration in the Planning Scene are separate states. This project includes checks and synchronization between them.

| Package | Contents |
| --- | --- |
| `my_arm_with_ball_description` | UR5/RG2 models, meshes, SDF/Xacro |
| `my_arm_with_ball_gazebo` | Gazebo worlds and C++ system plugins |
| `my_arm_with_ball_moveit_config` | MoveIt configuration, planning demo, planning validation tool |
| `my_arm_with_ball_application` | Keyboard control, grasp test, MTC pick and place |
| `my_arm_with_ball_bringup` | Integrated launch files, controller and bridge configuration |

### Usage

#### 1. Install and Build

The development environment is **Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic**. The commands below assume ROS 2 and Gazebo are already installed.

```bash
git clone https://github.com/dongseoki/my_arm_with_ball.git
cd my_arm_with_ball

source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
sudo apt install python3-colcon-common-extensions python3-rosdep

# Run only when setting up rosdep on this system for the first time
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Run the following launch commands from the repository root. Load the environment first in each new terminal.

```bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
source install/setup.bash
```

#### 2. Run Automated Pick and Place

Run the following command to watch the complete sequence. It also starts Gazebo and RViz.

```bash
ros2 launch my_arm_with_ball_bringup pick_and_place_test.launch.py headless:=false rviz:=true
```

- Success log: `STAGE5 PICK AND PLACE PASSED; waiting`
- Failure log: `STAGE5 FAILED stage=... reason=...; waiting without retry`
- The node and simulation stay running after success or failure for inspection. There is no automatic retry.
- Use `headless:=true rviz:=false` to run without a GUI.

Press `Ctrl+C` to stop. Start each simulation example **one at a time, after stopping the previous run**.

#### 3. Simulation and Manual Control

Start Gazebo, MoveIt, controllers, and RViz without the automated task. Use the RViz MotionPlanning panel to plan and execute motions.

```bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py
```

Add `headless:=true` to run without a GUI, or `rviz:=false` to disable only RViz.

For keyboard control, use a **separate interactive terminal** with the environment loaded.

```bash
ros2 run my_arm_with_ball_application keyboard_joint_teleop
```

| Key | Action |
| --- | --- |
| `1`–`6` | Select a UR5 joint |
| `a` / `d` | Change the selected joint target by −0.05 / +0.05 rad |
| `o` / `c` | Open / close the gripper |
| `r` | Move the selected arm joint to 0 rad |
| `x` | Move all arm joints to 0 rad; keep the gripper unchanged |
| `q` | Quit the keyboard node |

Keyboard control sends trajectories directly to the controllers and **bypasses MoveIt collision checking**. Additional motion inputs are ignored while a trajectory is running.

#### 4. Tests and Status Checks

**Grasp-only test:** Start a fresh simulation to validate grasping. On success, it prints `STAGE4 GRASP TEST PASSED` and shuts down automatically.

```bash
ros2 launch my_arm_with_ball_bringup pick_grasp_test.launch.py headless:=true rviz:=false
```

**Planning-only test:** Start the MoveIt demo without Gazebo, then run the validation tool in a separate terminal with the environment loaded. No robot trajectories are executed.

```bash
ros2 launch my_arm_with_ball_moveit_config demo.launch.py
```

```bash
ros2 run my_arm_with_ball_moveit_config validate_planning.py
```

**Controller status:** Check from a separate terminal while the integrated simulation is running.

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
```

**Package tests:** Run after building. These are separate from the simulation behavior tests above.

```bash
colcon test
colcon test-result --verbose
```

### Model Attribution

The UR5/RG2 model is based on [Gazebo Fuel's `anni/ur5_rg2`](https://fuel.gazebosim.org/1.0/anni/models/ur5_rg2/1).
