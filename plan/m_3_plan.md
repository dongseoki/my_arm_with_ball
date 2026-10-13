# MoveIt 적용 3단계 상세 계획

## 1. 목적과 범위

이 문서는 [`moveit_apply.md`](moveit_apply.md)의 3단계인 **Gazebo ros2_control과 MoveIt 실행 인터페이스 연결**을 구현 가능한 작업과 검증 기준으로 구체화한다. Gazebo와 MoveIt을 기존 `my_arm_with_ball_bringup/launch/simulation.launch.py`에서 함께 시작하고, 팔 6축은 `joint_trajectory_controller`의 `FollowJointTrajectory` 액션 하나로 제어한다. 기존 `arm_position_controller`는 제거해 동일 관절을 제어하는 중복 경로를 두지 않는다.

키보드 노드는 기존 키 조작을 유지하되, 팔 동작을 실제 `/joint_states` 기준 상대 목표로 계산하여 trajectory 액션에 직접 보낸다. **키보드 직접 제어는 MoveIt 경로 계획 및 충돌 검사를 거치지 않는다.** 반면 RViz/MoveIt에서 계획한 궤적은 MoveIt controller manager를 통해 같은 팔 trajectory 액션에 전달한다. 두 경로는 컨트롤러만 공유하며 목표 생성·검증 경로는 다르다.

그리퍼 `o/c`는 기존 `gripper_position_controller` 토픽 제어를 유지한다. 이번 단계에서는 그리퍼를 MoveIt 그룹으로 실행하지 않으며 접촉 감지, 공 부착/분리, 전체 Pick-and-Place 동작도 범위에서 제외한다. 이 단계가 완료되면 그리퍼의 MoveIt 실행 연결을 별도 단계로 추가하는 방안을 검토한 뒤 접촉·부착 작업으로 넘어간다.

## 2. 확정 기준

| 항목 | 단계 3 기준 |
|---|---|
| 환경 | Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic / MoveIt 2 Jazzy |
| 로봇 관절 상태 | Gazebo의 ros2_control이 측정 상태를 제공하고 `joint_state_broadcaster`가 `/joint_states`에 게시 |
| 팔 실행 컨트롤러 | `joint_trajectory_controller/JointTrajectoryController` 하나만 팔 6축을 제어 |
| 팔 액션 | 팔 trajectory 컨트롤러의 `FollowJointTrajectory` 액션 |
| 그리퍼 | 현재 `gripper_position_controller` 및 `o/c` 토픽 동작 유지 |
| MoveIt 시작 오차 | 실행 궤적 시작 상태와 Gazebo 실제 상태의 관절별 차이가 `0.02 rad`를 초과하면 실행 거부 |
| 키보드 입력 상태 | 입력 처리 직전에 받은 신선한 `/joint_states`를 기준으로 목표 계산; 상태가 `0.5 s`보다 오래됐으면 명령하지 않음 |
| 키 이동 | `a/d`는 선택 관절의 현재 측정값에서 각각 `-0.05/+0.05 rad`; URDF 관절 한계를 넘는 입력은 거부하고 오류 로그 기록 |
| 짧은 키보드 궤적 | `0.05 rad` 이동의 시작 실행 시간은 `1.0 s`; 실측 추종 결과를 보고 조정 가능 |
| 목표 도달 | 동작 후 `/joint_states`의 목표 오차가 관절별 `0.01 rad` 이내인지 확인 |
| 상태 확인 timeout | 궤적 예상 시간에 `2.0 s` 여유를 더해 action/상태 확인 timeout을 구성; 실제 구현 시 무한 대기가 없도록 한다 |
| 입력 중첩 | trajectory action 실행 중 팔 조작 키 입력은 무시하고 로그만 남김(사용자 별도 안내 없음) |
| 액션 실패 | 한국어와 영어를 함께 포함한 명시적 오류 로그; 실패 후 자동 재시도 없이 키보드 노드는 계속 실행 |
| 액션 timeout | 진행 중 goal 취소 요청, 실패 로그 기록, 취소 결과가 확인된 뒤 입력 재개; 노드는 계속 실행 |
| 리셋/그리퍼 키 | `r`은 선택 팔 관절을 0 rad로, `x`는 팔 6축만 0 rad로 이동; `x`는 그리퍼에 명령하지 않음. `o/c`는 기존 그리퍼 토픽 유지 |
| 통합 실행 | Gazebo, ros2_control, MoveIt `move_group`, MoveIt RViz를 `simulation.launch.py`에서 실행; 대화형 키보드 노드는 별도 터미널에서 실행 |

시작/목표 오차와 시간값은 단계 3의 보수적인 초기 검증 기준이다. 특히 MoveIt 자체의 시작 상태 허용 오차, trajectory controller action tolerance 및 `/joint_states` 최종 상태 판정은 각기 다른 검사점이므로 같은 이름의 값 하나로 대체하지 않는다. 구현 시 각 설정이 실제로 적용되는 지점을 확인하고 결과를 기록한다.

## 3. 완료 기준

다음 항목을 모두 만족하면 3단계를 완료한다.

1. `simulation.launch.py` 한 번으로 Gazebo, robot state publisher, `joint_state_broadcaster`, 팔 trajectory 컨트롤러, 그리퍼 position 컨트롤러, MoveIt `move_group`, MoveIt MotionPlanning RViz가 정상 시작된다. headless 실행에서는 RViz만 비활성화할 수 있다.
2. ros2_control에서 팔 trajectory 컨트롤러와 그리퍼 컨트롤러가 `active`이고, 팔 6축의 position 명령 인터페이스를 trajectory 컨트롤러만 점유한다. 팔을 제어하는 `arm_position_controller`는 등록되거나 실행되지 않는다.
3. 팔 trajectory 컨트롤러의 `FollowJointTrajectory` 액션 서버가 발견되고, MoveIt의 controller 설정이 정확한 액션 이름 및 6개 팔 관절 목록을 가리킨다. MoveIt trajectory execution이 허용되어 RViz/MoveIt에서 계획한 궤적이 Gazebo에 도달한다.
4. MoveIt 실행 전에 궤적 시작 상태와 Gazebo 실제 상태의 차이를 관절별로 검사한다. 불일치가 `0.02 rad` 초과인 경우 실행을 거부하고 원인을 로그에 남긴다.
5. 키보드 `1`~`6`의 관절 선택, `a/d`의 상대 `0.05 rad` 동작, `r`의 선택 관절 0 rad 리셋, `x`의 팔 전체 0 rad 리셋이 실제 `/joint_states`와 일치한다. 키보드 시작 시 팔에 초기 0 rad 명령을 자동 발행하지 않는다.
6. `a/d`로 요청한 목표는 매번 최신 상태에서 계산된다. 상태가 0.5초보다 오래됐거나 목표가 URDF 관절 한계를 넘으면 trajectory를 보내지 않는다. 목표 달성은 `/joint_states`의 관절별 `0.01 rad` 이내 오차로 검증한다.
7. `o/c`는 기존 그리퍼 토픽을 통해 정상 작동하고, `x`는 그리퍼 명령을 발행하지 않는다.
8. action 실행 중 추가 팔 키 입력이 무시되고 로그가 남는다. 액션 실패·거부·timeout·취소·컨트롤러 비활성·상태 미도달은 원인을 구별하여 한국어 및 영어가 함께 있는 오류 로그로 보고한다. 일반 실패 후 노드는 계속 입력을 받으며 자동 재시도는 하지 않는다.
9. timeout 시 현재 goal 취소를 요청한다. 취소 완료를 확인한 뒤에만 새 팔 명령을 허용하고, 액션 서버/상태 모니터 등 선행 서비스가 일정 시간 내 준비되지 않으면 키보드 노드는 명확한 오류를 남기고 종료한다.
10. 새 ROS 의존성을 package manifest에 선언하고 관련 패키지 빌드, 테스트 및 lint를 통과시킨다. 실제 headless Gazebo 통합 실행 결과와 재현 절차가 계획 결과에 기록된다.

## 4. 예상 변경·생성 파일

아래는 현재 코드 구조와 합의된 범위에 근거한 예상 파일 목록이다. 구현 중 실제 생성된 launch/test 파일이 기존 구성에 자연스럽게 포함될 수 있으면 불필요한 파일 추가보다 기존 파일 확장을 우선한다.

| 파일 | 예상 변경 유형 | 변경 사항(항목별 1줄 요약) |
|---|---|---|
| `my_arm_with_ball_bringup/config/ros2_controllers.yaml` | 수정 | 팔 `ForwardCommandController`를 6축 `joint_trajectory_controller`로 교체하고, 그리퍼 position 컨트롤러 설정은 유지한다. |
| `my_arm_with_ball_bringup/launch/simulation.launch.py` | 수정 | 기존 Gazebo launch에 MoveIt 실행을 통합하고, broadcaster 성공 후 arm trajectory 및 gripper 컨트롤러를 순서대로 활성화한다. |
| `my_arm_with_ball_bringup/package.xml` | 수정 | trajectory controller 및 통합 MoveIt launch에 필요한 실행 의존성을 선언한다. |
| `my_arm_with_ball_moveit_config/config/moveit_controllers.yaml` | 수정 | MoveIt controller manager에 팔 컨트롤러 이름, `FollowJointTrajectory` 액션 namespace, 팔 6관절 매핑을 등록한다. |
| `my_arm_with_ball_moveit_config/launch/move_group.launch.py` | 수정 | 계획 전용 실행 제한을 통합 실행에서도 사용할 수 있도록 실행 허용/비허용 launch 설정으로 분리하고 기존 planning-only 동작을 보존한다. |
| `my_arm_with_ball_moveit_config/config/joint_limits.yaml` | 검토/수정 가능 | 실제 trajectory 실행에 적용되는 속도·가속도 제한이 보수적이고 URDF 한계보다 느슨하지 않은지 검증해 필요한 값만 반영한다. |
| `my_arm_with_ball_moveit_config/package.xml` | 수정 가능 | 통합 MoveIt launch에서 직접 사용하는 의존성이 빠진 경우에만 선언한다. |
| `my_arm_with_ball_application/scripts/keyboard_joint_teleop.py` | 수정 | Float64 팔 토픽 발행을 action client 실행으로 바꾸고 실제 관절 상태·입력 중첩·timeout·실패 처리를 구현하며 그리퍼 토픽은 유지한다. |
| `my_arm_with_ball_application/package.xml` | 수정 | `control_msgs`, `trajectory_msgs`, `sensor_msgs` 등 action 및 실제 관절 상태 구독에 직접 필요한 실행 의존성을 선언한다. |
| `my_arm_with_ball_application/test/test_keyboard_joint_teleop.py` | 수정 | 상대 목표, 관절 한계, 상태 신선도, 액션 성공/실패/timeout, 입력 무시, `r/x` 및 그리퍼 보존 동작을 단위 검증한다. |
| `my_arm_with_ball_bringup/test/test_simulation_launch.py` | 생성 가능 | 실행 구성, 조건부 RViz, 계획 전용/통합 launch 및 컨트롤러 순서를 정적으로 검사할 필요가 있을 때 테스트를 추가한다. |
| `my_arm_with_ball_bringup/README.md` 또는 `README.md` | 수정 가능 | 통합 launch 및 별도 키보드 실행 명령이 기존 사용 안내와 달라질 때 재현 가능한 실행 절차를 갱신한다. |
| `plan/moveit_apply.md` | 수정 | 단계 3의 상태와 완료 결과를 실제 구현·검증 결과에 따라 반영한다. |
| `plan/m_3_plan.md` | 수정 | 구현 결과, 검증 로그, 수치 조정 및 남은 제한을 계획서에 기록한다. |

`my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro`의 팔 position command interface는 새 trajectory controller도 그대로 사용하므로 인터페이스 변경 대상으로 예상하지 않는다. 그리퍼 ros2_control 구성은 기존 controller를 유지한다. 충돌하는 예전 실행 경로/문서가 확인되면 팔 컨트롤러 이름을 참조하는 항목만 정합성 있게 갱신한다.

## 5. 구현 순서

### 5.1 선행 기준 및 분리 실행 보존

1. ROS 2 Jazzy에 설치된 `joint_trajectory_controller`, FollowJointTrajectory action, MoveIt controller plugin의 실제 이름과 action endpoint 구성을 확인한다.
2. 현재 planning-only `demo.launch.py`와 `move_group.launch.py`의 실행 비활성 경계를 보존한다. Gazebo 통합 실행에서만 trajectory 실행을 허용하는 launch 인자/구성을 정한다.
3. 현재 `/joint_states`가 8개 관절을 제공하고, 팔 trajectory controller와 그리퍼 controller가 서로 다른 명령 인터페이스를 사용하게 될 구조를 확인한다.

### 5.2 ros2_control trajectory controller

1. `ros2_controllers.yaml`에서 팔 controller type을 `joint_trajectory_controller/JointTrajectoryController`로 변경하고 6축 이름·순서·position command 및 position/velocity state 인터페이스를 설정한다.
2. 그리퍼는 기존 `ForwardCommandController`로 둔다. `forward_command_controller` 의존성을 제거하지 않는다.
3. launch spawner 순서를 `joint_state_broadcaster` 성공 → arm trajectory controller 성공 → gripper position controller로 구성하고 실패 단계별로 화면 로그를 남긴다.
4. `ros2 control list_controllers`, `list_hardware_interfaces`, `ros2 action list/info`로 활성 상태, 인터페이스 점유, action endpoint를 각각 검증한다.

### 5.3 MoveIt 실행 연결 및 통합 launch

1. MoveIt controller manager 설정에 arm trajectory controller 이름, action namespace, `FollowJointTrajectory` 타입, `ur5` 그룹의 6개 관절을 기록한다. 그리퍼 실행 controller는 이번 단계에서 MoveIt 목록에 넣지 않는다.
2. 통합 Gazebo launch가 `move_group`과 MotionPlanning RViz를 시작하도록 하되, 기존 RViz 인자 의미 및 `headless:=true` 동작을 유지한다. RViz에서는 MoveIt MotionPlanning 설정을 사용한다.
3. 계획 전용 실행과 통합 실행 설정이 충돌하지 않도록 한다. standalone planning demo는 계속 궤적 실행을 비활성화하고, Gazebo 통합에서만 실제 실행을 허용한다.
4. 통합 실행 중 실제 `/joint_states`가 MoveIt current state monitor에 들어오는지, planning frame 및 robot description이 일치하는지 확인한다.
5. MoveIt에서 작은 검증 계획을 실행한다. 시작 상태가 실제 Gazebo 상태와 `0.02 rad` 이내인지 확인하고, 불일치 시 실행이 차단되는지도 검증한다.

### 5.4 키보드 action 경로

1. 키보드 시작 시 arm trajectory controller/action 및 gripper position controller 활성화를 기다리되, 팔이나 그리퍼에 초기 자세 명령을 자동 발행하지 않는다. 시작 시 상태가 유효하지 않으면 한국어·영어 오류를 남기고 노드를 종료한다.
2. `/joint_states`를 구독해 6개 팔 관절의 최신 상태를 이름으로 매핑한다. 메시지 누락 관절, 잘못된 값, 오래된 상태(0.5초 초과)는 명령 전 오류로 처리한다.
3. `a/d`는 선택된 관절의 현재 측정값에 `±0.05 rad`를 더해 목표를 계산하고, URDF 한계를 넘으면 clamp하지 않고 동작을 거부하며 로그를 남긴다. 궤적 목표는 한 지점, 기본 `time_from_start=1.0 s`로 보낸다.
4. `1`~`6`은 선택 관절만 바꾸고, `r`은 선택 관절 0 rad, `x`는 팔 전체 0 rad 목표로 action을 보낸다. 큰 리셋 이동은 `1.0 s`를 고정 적용하지 않고 최대 관절 이동량과 보수적 velocity/acceleration scale에 비례하는 시간을 정한다.
5. 팔 action 진행 중 들어온 팔 조작 입력은 실행 큐에 넣거나 합치지 않고 무시하며, 무시 사실을 로그에 남긴다. `o/c`는 현재 그리퍼 토픽 경로로 유지한다.
6. 액션 종료 뒤 실제 `/joint_states` 목표 오차를 관절별 `0.01 rad`와 비교한다. action success인데 실제 상태가 오차를 벗어나면 별도의 상태 미도달 실패로 처리한다.
7. 일반 거부·실패는 한국어/영어 원인 로그를 남기고 새 키 입력 대기로 복귀한다. timeout에서는 진행 중 goal 취소를 요청하고 취소 완료 후 재개하며 자동 재시도하지 않는다.

### 5.5 오류 메시지와 안전 경계

로그는 한 이벤트마다 한국어와 영어를 함께 포함하고, 적어도 실패 종류, 컨트롤러/action 이름, 영향을 받은 관절, timeout 또는 결과 코드, 다음 동작(입력 재개/노드 종료)을 알 수 있게 한다. 예상 실패에는 traceback을 반복 출력하지 않고, 예외 세부 정보는 디버깅에 필요한 경우 추가한다.

키보드 직접 액션은 MoveIt collision checking을 우회하므로 단계 검증은 공/바구니에 접근하지 않는 시작 자세 주변의 작은 팔 움직임만 사용한다. MoveIt 실행 검증도 경로를 계획하고 충돌 확인한 뒤 수행한다. 테스트 시작 전 Gazebo 실제 상태, 컨트롤러 활성 상태, 관절 한계 및 입력 관절 목록을 확인한다.

## 6. 검증 계획

### 6.1 정적/단위 검증

- Python 키보드 단위 테스트에서 실제 상태로부터의 상대 목표, `0.05 rad` step, 리셋 목표, 한계 초과 거부, 상태 누락/신선도, busy 입력 무시, action 성공/거부/실패/timeout 및 취소 후 재개를 검증한다.
- `x`가 팔 action만 요청하고 그리퍼 publisher를 호출하지 않으며 `o/c`는 기존 두 값 그리퍼 명령을 내는지 검사한다.
- YAML 파싱과 controller joint 목록 중복/불일치 검사, MoveIt mapping과 action/controller 이름 일치, 통합 launch 인자 및 planning-only 제한 보존을 검사한다.
- `colcon build --symlink-install`, 패키지별 `colcon test`, `colcon test-result --verbose`, ament lint를 수행한다.

### 6.2 Gazebo 통합 검증

1. `ros2 launch my_arm_with_ball_bringup simulation.launch.py headless:=true`로 실행한다.
2. trajectory arm controller, gripper position controller, joint state broadcaster의 `active` 상태와 팔 command interface의 단일 controller 점유를 확인한다. 팔 position controller가 나타나지 않아야 한다.
3. 액션 서버와 MoveIt current state를 확인한다. `/joint_states`는 한 publisher로부터 8개 관절의 이름·상태를 제공해야 한다.
4. 초기 상태 부근의 MoveIt 계획 경로를 한 번 실행하여 controller 결과와 `/joint_states` 추종을 확인한다. 시작 상태를 의도적으로 threshold 밖으로 바꾼 경우 실행 거부를 검증한다.
5. 별도 터미널에서 `ros2 run my_arm_with_ball_application keyboard_joint_teleop`를 실행한다. `1d`, `a`, `r`, `x`를 사용해 현재 측정 기준 `+0.05 rad`, 선택 관절 0, 팔 전체 0 이동을 확인하고 각 종료 상태가 `0.01 rad` 허용 오차 이내인지 측정한다.
6. 실행 중 입력, 관절 한계 초과 입력, 오래된 상태, 액션 거부/비활성, 실행 timeout을 각기 검증한다. timeout에서는 취소 완료 전에 새 goal이 보내지 않는지 확인한다.
7. `o/c`로 기존 그리퍼 동작이 유지되고, `x`가 그리퍼를 닫거나 움직이지 않는지 확인한다.
8. RViz의 MoveIt MotionPlanning에서 짧은 충돌 없는 경로를 계획·실행해 Gazebo와 `/joint_states`가 추종하는지 확인한다.

화면 기반 확인이 가능한 경우 RViz 및 Gazebo 움직임을 확인한다. 헤드리스 검증에서는 action result와 `/joint_states` 값을 핵심 증거로 남기고, 시각적 확인을 했다고 주장하지 않는다.

## 7. 위험과 완화

| 위험 | 완화 |
|---|---|
| 기존 키보드가 내부 0 rad 목표값을 사용해 실제 자세와 어긋난 명령을 보냄 | 매 명령 직전 최신 `/joint_states`로 상대 목표를 만들고 자동 초기 명령을 제거한다. |
| trajectory controller 전환 중 구 controller가 함께 실행되어 같은 관절을 점유 | 구 arm controller 설정을 제거하고 인터페이스 점유 및 active controller 목록으로 단일 소유를 확인한다. |
| 키보드 action이 MoveIt 경로 계획을 우회해 물체/로봇과 충돌 | 키보드 검증은 시작 자세 근처의 작은 이동으로 제한하고 이 차이를 문서와 실행 도움말에 명시한다. |
| 통합 launch와 planning-only demo 설정이 섞여 Gazebo 없는 실행에서 실제 실행이 열림 | 실행 허용을 명시적인 launch 경계로 분리하고 standalone demo의 비활성 회귀 검사를 추가한다. |
| timeout 뒤 controller가 이전 목표를 계속 실행하는 동안 새 명령이 도착 | cancel 완료 전 새 action goal을 금지하고 결과가 확인되지 않으면 오류를 크게 기록한다. 구현상 취소 확인이 불가능한 상태에서는 새 팔 명령을 보내지 않는 안전 차단을 유지한다. |
| action 결과는 성공이나 Gazebo 물리 상태가 목표를 따라오지 않음 | action 성공과 별개로 `/joint_states` 목표 오차를 측정하고 허용 오차 미달을 별도 실패로 처리한다. |
| RViz 실행 설정 및 기존 시뮬레이션 RViz가 중복/서로 다른 robot description을 게시 | 통합 launch에서 MoveIt MotionPlanning RViz를 하나만 실행하고 기존 RViz 설정의 독립 실행 동작은 별도로 보존한다. |

## 8. 범위 밖 및 다음 단계 인계

- 그리퍼 MoveIt controller/action 및 gripper group 실행 연결은 이번 단계에 포함하지 않는다. `o/c`는 기존 직접 토픽 제어를 보존한다.
- Gazebo 접촉 감지, 사과의 그리퍼 부착/분리, Planning Scene attached object 갱신은 포함하지 않는다.
- 공 접근·집기·lift·바구니 이동·놓기 전체 task/action 및 자동 재시도는 포함하지 않는다.
- 별도 후속 단계에서 그리퍼를 MoveIt으로 구동할 경우, 그리퍼 trajectory/action 연결과 실패/상태 검증을 먼저 별도 검증한 다음 기존 접촉·부착 단계에 합류하는 순서를 검토한다.

완료 시 실제 파일 변경, 사용한 실행 명령, 컨트롤러 및 action 이름, 각 테스트 결과, 기준값 조정 근거, 미해결 제한을 이 문서와 [`moveit_apply.md`](moveit_apply.md)에 기록한다.
