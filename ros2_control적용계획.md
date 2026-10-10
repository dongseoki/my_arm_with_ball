# ros2_control 적용 계획

## 1. 목표와 진행 방식

현재 Gazebo에서 동작하는 UR5 팔의 키보드 제어를 ros2_control로 전환하고, 실습 과정에서 제어 구조를 이해한다.

사용자와 합의한 진행 방식은 **설명 → 작은 변경 → 실행 결과 확인**이다. 전체 구현을 한 번에 끝내지 않고 단계별로 변경 이유, 명령 흐름, 확인 방법을 설명한다. 실행 결과를 함께 확인한 뒤 다음 단계로 진행하며, 구현 중 중요한 모호함이 발견되면 사용자와 논의한다.

## 2. 합의한 적용 범위

- UR5 팔의 6개 관절에 ros2_control을 적용한다.
- RG2 그리퍼의 명령은 기존 제어 방식을 유지하고, 상태만 ros2_control의 읽기 전용 인터페이스에 포함한다.
- 키보드의 `1~6`, `a/d`, `r/x` 조작을 유지한다.
- 그리퍼의 `o/c` 조작을 유지한다.
- MoveIt 적용, 실제 로봇 연결, 그리퍼 명령의 ros2_control 전환은 이번 단계의 범위에 포함하지 않는다.
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

## 3. 전환 전 프로젝트 구조

현재 로봇은 `my_arm_with_ball_description/models/ur5_rg2/model.sdf`에 정의되어 있으며, Gazebo 월드에서 불러온다.

팔과 그리퍼의 현재 명령 흐름:

```text
keyboard_joint_teleop
  → 관절별 Float64 ROS 명령 토픽
  → ROS–Gazebo 브리지
  → Gazebo 관절 제어
```

현재 관절 상태 흐름:

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
- 모델: 기존 물리 SDF와 world 고정을 유지하고, 파서에 맞는 제어용 설명을 별도로 제공한다. 전체 URDF 변환을 필수로 가정하지 않는다.
- 독점 제어: ros2_control 활성화 전에 기존 팔 Gazebo 제어 플러그인 6개를 비활성화한다. 그리퍼 플러그인 2개는 유지한다.
- 상태: 팔 6개와 그리퍼 2개의 실제 상태를 하나의 broadcaster로 제공하고 기존 `/joint_states` 브리지를 대체해 중복을 방지한다.

설명 파싱은 원본 SDF에서 world 고정 조인트를 제외한 **설명용 데이터**로 확인했다. 원본 물리 모델의 고정 조인트는 제거하지 않는다. 실제 제어 자원 파싱, Gazebo 연결 및 그리퍼 상태 공존은 2단계에서 검증한다.

## 5. 단계별 실습 계획

### 0단계: 환경과 기존 동작 확인

- ROS 2 및 Gazebo 버전과 필요한 연동 패키지의 설치 상태를 확인한다.
- 기존 시뮬레이션의 실행 방법과 팔·그리퍼 제어 경로를 확인한다.
- `1~6`, `a/d`, `r/x`, `o/c`의 동작을 기준으로 기록한다.
- 기존 문서에 기록된 초기 자세와 `x` 재설정 자세의 차이를 확인한다. 현재 코드의 재설정 목표는 팔의 모든 관절 `0.0` rad이며, 이것이 시뮬레이션 최초 자세와 같다고 가정하지 않는다.

확인 결과: 적용 환경, 기존 동작, 초기화 기준을 설명할 수 있다.

### 1단계: 제어 구성 결정 — 완료 (2026-10-10)

- [x] rrbot 실습에서 배운 하드웨어 인터페이스, controller manager, 컨트롤러의 역할을 현재 프로젝트에 연결해 설명한다.
- [x] 키보드 위치 제어에 사용할 컨트롤러와 명령 형식을 정한다.
- [x] 현재 SDF 모델을 어떻게 ros2_control과 연결할지 검토한다. 설명 파일 추가 또는 변환이 필요하면 변경 범위와 이유를 먼저 설명한다.
- [x] 팔의 기존 Gazebo 제어와 ros2_control이 같은 관절을 동시에 제어하지 않도록 전환 방법을 정한다.
- [x] `/joint_states`에 팔과 그리퍼 상태를 제공하는 방법을 정한다. 중복 발행이나 관절 누락 방지 정책을 정한다. 실제 발행은 2단계에서 확인한다.

확인 결과: 팔 명령, 실제 상태, 그리퍼 명령의 전체 흐름과 필요한 변경을 이해한다.

### 2단계: ros2_control 설정과 Gazebo 연결 — 완료 (2026-10-10)

- [x] 6개 팔 관절의 명령·상태 인터페이스와 그리퍼 2개의 읽기 전용 상태 인터페이스를 정의한다.
- [x] 선택한 Gazebo 연동 구성과 컨트롤러 설정을 추가한다.
- [x] launch에 컨트롤러 초기화·활성화 절차를 연결한다.
- [x] 필요한 의존성과 설명·설정 파일 설치 규칙을 반영한다.
- [x] 실행 후 하드웨어 인터페이스와 컨트롤러 상태를 확인한다.

확인 결과: 대상 6개 관절이 인식되고 필요한 컨트롤러가 활성화된다.

#### 2단계 구현과 실행 검증 결과

- `model.sdf`: 팔 6개 `position` 명령, 팔·그리퍼 8개 `position`/`velocity` 상태를 정의했다. 기존 팔 제어 플러그인 6개와 기존 Gazebo 상태 발행 플러그인은 제거했다. 그리퍼 제어 플러그인 2개, 링크 형상·관성·관절 배치·제한 및 world 고정은 유지했다.
- `config/ros2_controllers.yaml`: 100 Hz controller manager, `arm_position_controller`, `joint_state_broadcaster`를 설정했다.
- `simulation.launch.py`: 원본 SDF에서 제어용 설명과 실행용 월드를 생성한다. 설명에서만 world 고정 조인트와 Gazebo 플러그인을 제외하고, 필수 `ros2_control/hardware/plugin`은 보존한다. 실행용 월드는 world 고정을 유지하며 연동 플러그인에 설치 위치에서 찾은 YAML 경로를 넣는다. 임시 월드는 종료 시 정리한다.
- launch 순서: 설명 발행 및 Gazebo 시작 → 상태 broadcaster 활성화 → 팔 controller 활성화. spawner 실패 시 후속 실행 없이 종료한다. `headless` 옵션은 GUI 없이 동일 구성을 검증하기 위한 것이다.
- `bridge.yaml`: 기존 `/joint_states` 연결은 제거하고, 그리퍼 명령과 `/clock` 연결은 유지했다. 사용하지 않는 기존 팔 명령 브리지 정리는 3단계에 남겼다.
- 빌드: `colcon build --symlink-install --packages-up-to my_arm_with_ball_bringup`으로 4개 패키지 빌드 통과.
- 회귀 검사: 인터페이스 분리, 물리 모델 보존, 설명 생성·경로 이동·원본 비변경, 필수 하드웨어 플러그인 보존, spawner 실패 처리 등 16개 테스트 통과. colcon/CTest 등록 검사도 통과.
- 실제 headless 실행: 두 컨트롤러 `active`, 팔 6개 `position` 명령 인터페이스 `available`/`claimed`, 8개 관절의 상태 인터페이스 16개 확인.
- `/joint_states`: 관절 이름 8개에 중복·누락이 없고 발행자는 `joint_state_broadcaster` 하나임을 확인. 메시지 시간 진행과 유한한 실제 위치·속도값 확인.
- 수동 팔 명령: 6개 배열 발행으로 `wrist_3_joint` 목표 `0.15` rad에 `0.01` rad 이내 도달하고, 전체 0 목표로 복귀함을 확인.
- 기존 그리퍼 토픽: 두 관절에 `1.18`/`0.0` 명령을 보내 열림·닫힘 방향으로 실제 상태가 변하는 것을 확인. 그리퍼의 ros2_control 명령 인터페이스는 없음.

실행과 확인 명령:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py rviz:=false headless:=true
```

별도 터미널에서도 같은 환경을 source한 뒤 확인한다. GUI를 보려면 `headless:=true`를 생략한다. 시뮬레이션은 컨트롤러 초기화를 위해 실행 상태로 시작한다.

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic info /joint_states -v
ros2 topic echo /joint_states --once
ros2 topic pub --once /arm_position_controller/commands std_msgs/msg/Float64MultiArray \
  '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.15]}'
```

**아직 하지 않은 것:** 팔 키보드 명령은 기존 토픽을 사용하는 코드 그대로이며 새 컨트롤러에 연결되지 않았다. 시작 시 0 목표, `a/d`, `r/x`의 새 배열 명령 연결은 3단계에서 구현한다. 키보드 전체 사용성·GUI 시각 확인·반복 실행의 통합 검증은 3~4단계에서 수행한다. 현재 controller manager의 명령 제한 강제는 기본값으로 비활성화되어 있으므로 수동 배열도 기존 관절 제한을 지킨다. 키보드 제한 검사는 3단계에서 유지한다.

실행 시 SDF의 사용자 정의 `ros2_control` 태그, KDL 루트 관성, 100 Hz 제어 주기와 1 ms 물리 주기의 차이에 대한 경고가 남는다. 이번 실행에서는 초기화·제어·상태 확인을 통과했다. 외부 모델의 `texture.png` 해석 경고도 관찰했으며 GUI 텍스처 표시는 이번 headless 검증 범위가 아니다.

### 3단계: 팔 키보드 명령 전환

- 팔 명령을 선택한 ros2_control 컨트롤러 형식으로 전환한다.
- 관절 선택, `0.05` rad 단위 증감, 관절 제한, 재설정 동작을 유지한다.
- 그리퍼의 기존 명령 토픽과 `o/c` 동작을 유지한다.
- `x`는 기존처럼 팔 전체와 그리퍼를 재설정하도록 유지한다.
- 팔의 기존 명령 브리지와 Gazebo 제어 설정 중 불필요해진 부분을 정리한다.

확인 결과: 기존 키보드 사용 방식으로 ros2_control을 거쳐 팔을 움직일 수 있다.

### 4단계: 통합 검증과 실습 정리

- 6개 관절을 각각 선택해 `a/d` 움직임을 확인한다.
- `r`과 `x`의 재설정 결과를 확인한다.
- 실제 팔 관절 위치가 `/joint_states`에 반영되는지 확인한다.
- 그리퍼 `o/c`가 기존처럼 동작하는지 확인한다.
- 시뮬레이션을 다시 실행해 컨트롤러 활성화와 키보드 조작이 재현되는지 확인한다.
- 변경한 패키지의 빌드와 관련 검사를 실행하고, 실행 방법 및 확인 명령을 정리한다.

확인 결과: 아래 완료 기준을 만족하고 사용자가 제어 흐름을 설명할 수 있다.

## 6. 합의한 완료 기준

- [ ] 키보드 `1~6`으로 UR5의 6개 관절을 선택할 수 있다.
- [ ] `a/d`로 선택한 관절의 목표 위치를 변경할 수 있다.
- [ ] `r`로 선택 관절을, `x`로 팔 전체와 그리퍼를 기존 재설정 목표로 돌릴 수 있다.
- [ ] 팔 명령이 ros2_control을 거쳐 Gazebo에 전달된다.
- [ ] 실제 팔 관절 위치가 `/joint_states`로 제공된다.
- [ ] 그리퍼의 `o/c` 조작은 기존 제어 방식으로 동작한다.
- [ ] 단계별 설명과 실행 확인을 통해 제어 구조를 함께 이해한다.

## 7. 구현 전 확인할 기술 사항

다음 항목은 사용자에게 환경 정보를 추측하게 하기보다 코드와 설치 환경을 직접 확인해 구체화한다. 선택에 따라 학습 범위나 기존 동작이 달라지는 경우 사용자와 논의한다.

- 설치된 ROS 2/Gazebo 조합에 맞는 연동 패키지와 설정 방식
- 현재 SDF 모델을 유지하거나 로봇 설명을 추가해야 하는지
- 키보드 위치 명령에 적합한 컨트롤러 및 명령 메시지
- 초기 목표 위치와 실제 초기 위치의 처리
- 팔과 그리퍼의 관절 상태를 `/joint_states`에 제공하는 방식

현재 상태: 1단계 제어 구성 결정과 2단계 ros2_control 설정·Gazebo 연결 완료. 키보드 팔 명령 전환(3단계)은 아직 시작하지 않았다.
