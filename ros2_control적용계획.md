# ros2_control 적용 계획

진행 현황: **0·1·2·3·4단계 기술 검증 완료, 5단계 그리퍼 전환 미완료**.
Gazebo에서 두 컨트롤러의 활성화, 키보드 팔 명령의 실제 이동, 그리퍼 `o/c` 동작과 8개 관절의 `/joint_states` 발행을 확인했다.

## 1. 목표와 진행 방식

현재 Gazebo에서 동작하는 UR5 팔의 키보드 제어를 ros2_control로 전환하고, 실습 과정에서 제어 구조를 이해한다.

사용자와 합의한 진행 방식은 **설명 → 작은 변경 → 실행 결과 확인**이다. 전체 구현을 한 번에 끝내지 않고 단계별로 변경 이유, 명령 흐름, 확인 방법을 설명한다. 실행 결과를 함께 확인한 뒤 다음 단계로 진행하며, 구현 중 중요한 모호함이 발견되면 사용자와 논의한다.

## 2. 합의한 적용 범위

- UR5 팔의 6개 관절에 ros2_control을 적용한다.
- RG2 그리퍼의 명령은 기존 제어 방식을 유지하고, 상태만 ros2_control의 읽기 전용 인터페이스에 포함한다.
- 키보드의 `1~6`, `a/d`, `r/x` 조작을 유지한다.
- 그리퍼의 `o/c` 조작을 유지한다.
- MoveIt 적용, 실제 로봇 연결, 그리퍼 명령의 ros2_control 전환은 이번 단계의 범위에 포함하지 않는다.
- 그리퍼 명령의 ros2_control 전환은 현재 팔 전환 및 통합 검증을 마친 뒤 별도 5단계에서 진행한다.
- 공을 집어 바구니에 놓는 자동 동작은 이번 단계의 완료 조건에 포함하지 않는다.

대상 관절과 키보드 선택 번호:

| 번호 | 관절 이름 |
| --- | --- |
| 1 | `shoulder_pan_joint` |
| 2 | `shoulder_lift_joint` |
| 3 | `elbow_joint` |
| 4 | `wrist_1_joint` |
| 5 | `wrist_2_joint` |
| 6 | `wrist_3_joint` |

## 3. 전환 전 프로젝트 구조 (기록)

전환 전 로봇은 `my_arm_with_ball_description/models/ur5_rg2/model.sdf`에 정의되어 있었으며, Gazebo 월드에서 불러왔다. 현재도 Gazebo 물리 모델은 이 SDF를 사용하고, ROS의 `robot_description`은 별도 Xacro에서 생성한 URDF를 사용한다.

전환 전 팔과 그리퍼의 명령 흐름:

```text
keyboard_joint_teleop
  → 관절별 Float64 ROS 명령 토픽
  → ROS–Gazebo 브리지
  → Gazebo 관절 제어
```

전환 전 관절 상태 흐름:

```text
Gazebo 관절 상태
  → ROS–Gazebo 브리지
  → /joint_states
```

주요 확인 파일:

- `my_arm_with_ball_application/scripts/keyboard_joint_teleop.py`
- `my_arm_with_ball_bringup/launch/simulation.launch.py`
- `my_arm_with_ball_bringup/config/bridge.yaml`
- `my_arm_with_ball_description/models/ur5_rg2/model.sdf`
- `my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf`
- `docs/control_validation_checklist.md`

## 4. 목표 제어 구조

팔의 목표 명령 흐름:

```text
keyboard_joint_teleop
  → ros2_control 컨트롤러
  → Gazebo 연동 하드웨어 인터페이스
  → UR5의 6개 관절
```

팔과 그리퍼의 실제 관절 상태는 ros2_control의 상태 인터페이스를 통해 읽고, 하나의 `joint_state_broadcaster`가 8개 관절을 `/joint_states`로 제공한다. 그리퍼에는 명령 인터페이스를 추가하지 않으며, 그리퍼 명령은 기존 ROS–Gazebo 브리지 경로를 유지한다.

1단계에서 확정한 구성:

- 환경: ROS 2 Jazzy, Gazebo Harmonic (`gz-sim8`), 설치된 `gz_ros2_control` 사용.
- 하드웨어 인터페이스: `gz_ros2_control/GazeboSimSystem`.
- 팔 컨트롤러: `forward_command_controller/ForwardCommandController`, `position` 인터페이스.
- 팔 명령: `/arm_position_controller/commands`에 `std_msgs/msg/Float64MultiArray`로 위 표 순서의 6개 목표각 전달. 이동 시간은 지정하지 않는다.
- 초기화: 키보드 노드 시작 시 기존처럼 모든 팔 관절에 `0.0` rad 목표를 명령한다. `r/x` 목표와 기존 키 조작은 유지한다. 키보드 명령 전환은 3단계에서 구현한다.
- 모델: 기존 물리 SDF와 world 고정을 유지하고, 별도 Xacro가 생성한 URDF를 `robot_description`에 제공한다. SDF를 변환해 `robot_description`으로 사용하지 않는다.
- 독점 제어: ros2_control 활성화 전에 기존 팔 Gazebo 제어 플러그인 6개를 비활성화한다. 그리퍼 플러그인 2개는 유지한다.
- 상태: 팔 6개와 그리퍼 2개의 실제 상태를 하나의 broadcaster로 제공하고 기존 `/joint_states` 브리지를 대체해 중복을 방지한다. Gazebo 연결과 그리퍼 상태 공존은 2단계에서 검증했다.

## 5. 단계별 실습 계획

### 0단계: 환경과 기존 동작 확인

- [x] ROS 2 및 Gazebo 버전과 필요한 연동 패키지의 설치 상태를 확인한다.
- [x] 기존 시뮬레이션의 실행 방법과 팔·그리퍼 제어 경로를 확인한다.
- [x] `1~6`, `a/d`, `r/x`, `o/c`의 동작을 기준으로 기록한다.
- [x] 기존 문서에 기록된 초기 자세와 `x` 재설정 자세의 차이를 확인한다. 현재 코드의 재설정 목표는 팔의 모든 관절 `0.0` rad이며, 이것이 시뮬레이션 최초 자세와 같다고 가정하지 않는다.

확인 결과: 적용 환경, 기존 동작, 초기화 기준을 설명할 수 있다.

### 1단계: 제어 구성 결정

- [x] rrbot 실습에서 배운 하드웨어 인터페이스, controller manager, 컨트롤러의 역할을 현재 프로젝트에 연결해 설명한다.
- [x] 키보드 위치 제어에 사용할 컨트롤러와 명령 형식을 정한다.
- [x] 현재 SDF 모델을 어떻게 ros2_control과 연결할지 검토한다. SDF와 별도 Xacro/URDF의 역할을 정한다.
- [x] 팔의 기존 Gazebo 제어와 ros2_control이 같은 관절을 동시에 제어하지 않도록 전환 방법을 정한다.
- [x] `/joint_states`에 팔과 그리퍼 상태를 제공하는 방법을 정하고, 발행자 1개와 8개 관절을 확인한다.

확인 결과: 팔 명령, 실제 상태, 그리퍼 명령의 전체 흐름과 필요한 변경을 이해한다.

### 2단계: ros2_control 설정과 Gazebo 연결

- [x] 6개 팔 관절의 명령·상태 인터페이스를 정의한다.
- [x] 선택한 Gazebo 연동 구성과 컨트롤러 설정을 추가한다.
- [x] launch에 컨트롤러 초기화·활성화 절차를 연결한다.
- [x] 필요한 의존성과 설명·설정 파일 설치 규칙을 반영한다.
- [x] 실행 후 하드웨어 인터페이스와 컨트롤러 상태를 확인한다.

확인 결과: 대상 6개 관절이 인식되고 필요한 컨트롤러가 활성화된다.

### 3단계: 팔 키보드 명령 전환

- [x] 팔 명령을 선택한 ros2_control 컨트롤러 형식으로 전환한다.
- [x] 관절 선택, `0.05` rad 단위 증감, 관절 제한, 재설정 동작을 유지한다.
- [x] 그리퍼의 기존 명령 토픽과 `o/c` 동작을 유지한다.
- [x] `x`는 기존처럼 팔 전체와 그리퍼를 재설정하도록 유지한다.
- [x] 팔의 기존 명령 브리지와 Gazebo 제어 설정 중 불필요해진 부분을 정리한다.

확인 결과: 기존 키보드 사용 방식으로 ros2_control을 거쳐 팔을 움직일 수 있다.
Gazebo에서 `1~6` 관절 선택과 명령·재설정, 그리퍼 `o/c`의 실제 이동을 확인했다. 키보드 노드를 시뮬레이션보다 먼저 시작해도 컨트롤러 활성화 후 초기 팔 명령이 전달됨을 확인했다.

### 4단계: 통합 검증과 실습 정리

- [x] 6개 관절을 각각 선택해 `a/d` 움직임을 확인한다.
- [x] `r`과 `x`의 재설정 결과를 확인한다.
- [x] 실제 팔 관절 위치가 `/joint_states`에 반영되는지 확인한다.
- [x] 그리퍼 `o/c`가 기존처럼 동작하는지 확인한다.
- [x] 시뮬레이션을 다시 실행해 컨트롤러 활성화와 키보드 조작이 재현되는지 확인한다.
- [x] 변경한 패키지의 빌드와 관련 검사를 실행하고, 실행 방법 및 확인 명령을 정리한다.

확인 결과 (2026-10-12, `headless:=true`): 키보드 `1~6`에서 각 관절의 `d` 입력 후 실제 위치가 약 `0.05` rad, `a` 입력 후 약 `0` rad로 돌아왔다. `r`은 선택한 관절을 `0.05 → 0` rad로, `x`는 팔과 그리퍼를 초기 목표인 `0`으로 되돌렸다. `o`에서는 두 그리퍼 관절이 약 `1.18` rad, `c`에서는 약 `0` rad였다. 이 값은 `/joint_states`에서 측정했다. 시뮬레이션을 종료하고 다시 실행한 뒤 두 컨트롤러가 `active`, `/joint_states` 발행자가 `joint_state_broadcaster` 하나임을 확인했고, 키보드 `1d` 입력의 `0 → 0.05` rad 이동도 재현했다. 헤드리스 실행이므로 화면에서 보이는 자세와 충돌 여부는 이 검증에 포함하지 않았다. 제어 흐름에 대한 사용자의 이해 여부는 함께 확인할 항목으로 남긴다.

실행 및 확인 명령 (프로젝트 루트):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_arm_with_ball_description my_arm_with_ball_gazebo my_arm_with_ball_bringup my_arm_with_ball_application --symlink-install
source install/setup.bash
colcon test --packages-select my_arm_with_ball_description my_arm_with_ball_gazebo my_arm_with_ball_bringup my_arm_with_ball_application
colcon test-result --verbose
ros2 launch my_arm_with_ball_bringup simulation.launch.py headless:=true rviz:=false
```

별도 터미널에서 같은 ROS 및 워크스페이스 환경을 적용한 뒤 `ros2 run my_arm_with_ball_application keyboard_joint_teleop`을 실행한다. 상태 확인에는 `ros2 control list_controllers`, `ros2 topic info /joint_states -v`, `ros2 topic echo /joint_states`를 사용한다. 이번 실행에서 네 패키지 빌드와 검사 3개가 모두 통과했다.

### 5단계: RG2 그리퍼 명령의 ros2_control 전환 (후속 작업)

- [ ] 두 그리퍼 관절의 명령 인터페이스와 컨트롤러를 정하고, 기존 `o/c`와 `x`의 목표값 및 관절 동작을 유지할 방법을 확인한다.
- [ ] Xacro/URDF에 두 관절의 명령 인터페이스를 추가하고 컨트롤러 설정과 활성화 절차를 연결한다.
- [ ] 같은 관절을 중복 제어하지 않도록 기존 Gazebo 그리퍼 제어 플러그인 2개와 명령 브리지 2개를 전환 시점에 제거한다.
- [ ] 키보드 그리퍼 명령을 새 컨트롤러에 연결하고 `o/c`, `x`, 시작 시 초기 명령을 확인한다.
- [ ] 두 관절의 실제 상태와 재시작 후 동작을 검증하고 실행 방법을 정리한다.

이 단계는 3단계 팔 키보드 전환과 4단계의 기존 구성 통합 검증이 끝난 뒤 착수한다. 현재 3·4단계의 완료 기준에는 포함하지 않는다.

## 6. 합의한 완료 기준

- [x] 키보드 `1~6`으로 UR5의 6개 관절을 선택할 수 있다.
- [x] `a/d`로 선택한 관절의 목표 위치를 변경할 수 있다.
- [x] `r`로 선택 관절을, `x`로 팔 전체와 그리퍼를 기존 재설정 목표로 돌릴 수 있다.
- [x] 팔 명령이 ros2_control을 거쳐 Gazebo에 전달된다. (컨트롤러 토픽 직접 발행으로 확인)
- [x] 실제 팔 관절 위치가 `/joint_states`로 제공된다.
- [x] 그리퍼의 `o/c` 조작은 기존 제어 방식으로 동작한다.
- [ ] 단계별 설명과 실행 확인을 통해 제어 구조를 함께 이해한다.

## 7. 기술 확인 사항

다음 항목은 코드와 설치 환경에서 확인한다.

- [x] 설치된 ROS 2/Gazebo 조합에 맞는 연동 패키지와 설정 방식
- [x] SDF 유지와 별도 Xacro/URDF 설명 추가 방식
- [x] 키보드 위치 명령에 적합한 컨트롤러 및 명령 메시지
- [x] 초기 목표 위치와 실제 초기 위치의 처리
- [x] 팔과 그리퍼의 관절 상태를 `/joint_states`에 제공하는 방식

현재 상태: SDF와 Xacro/URDF를 분리하고 `gz_ros2_control` 및 두 컨트롤러를 연결했다. 키보드의 팔 명령을 `/arm_position_controller/commands`로 전환했다. 초기 목표는 실제 시작 위치를 읽어 복사하지 않고 기존처럼 0 rad로 발행한다. 4단계 기술 검증을 마쳤으며, 그리퍼 명령 전환은 별도 5단계다.
