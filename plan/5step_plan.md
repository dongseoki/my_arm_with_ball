# ros2_control 적용 5단계 실행 계획: RG2 그리퍼 명령 전환

상태: **5단계 구현 및 헤드리스 기술 검증 완료 (2026-10-12).** 이 문서는 `plan/ros2_control적용계획.md`의 5단계만 다룬다. 변경 전 동작의 기준은 같은 문서의 4단계 검증 결과다.

## 목표와 범위

RG2의 `rg2_finger_joint1`, `rg2_finger_joint2` 위치 명령을 기존 Gazebo 관절 제어 플러그인과 ROS–Gazebo 브리지에서 ros2_control로 옮긴다. 키보드 `o/c/x`와 시작 시 초기 목표, UR5 팔의 `1~6`, `a/d`, `r/x` 동작을 유지한다. 실제 로봇 연결, MoveIt, 공 집기 자동 동작은 이 단계에 포함하지 않는다. 기존 목표값은 두 관절 모두 닫힘 `0.0` rad, 열림 `1.18` rad다. [기준값](../my_arm_with_ball_application/scripts/keyboard_joint_teleop.py#L33-L35), [현재 키 처리](../my_arm_with_ball_application/scripts/keyboard_joint_teleop.py#L173-L178)

## 결정된 제어 방식

- 두 손가락 관절에 `position` 명령 인터페이스를 추가한다. 기존 `position`, `velocity` 상태 인터페이스와 하나의 `joint_state_broadcaster`는 유지한다. [현재 제어 정의](../my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro#L150-L173)
- 팔과 같은 `forward_command_controller/ForwardCommandController`를 별도의 `gripper_position_controller`로 사용한다. 명령은 `/gripper_position_controller/commands`의 `std_msgs/msg/Float64MultiArray`에 **관절 1, 관절 2 순서로 정확히 두 값**을 넣는다. `o`는 `[1.18, 1.18]`, `c`와 `x` 및 노드 시작은 `[0.0, 0.0]`이다. 팔 컨트롤러와 그리퍼 컨트롤러는 서로 다른 관절의 `position` 인터페이스를 점유한다. [현재 팔 설정](../my_arm_with_ball_bringup/config/ros2_controllers.yaml#L1-L18), [관절 제한](../my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro#L134-L144)
- 키보드 노드는 **팔과 그리퍼 컨트롤러가 모두 `active`인 뒤** 두 초기 명령을 발행하고 키 입력을 받는다. 시뮬레이션보다 먼저 실행한 경우에도 초기 그리퍼 명령이 유실되지 않도록 기존 120초 대기와 실패 시 종료 동작을 두 컨트롤러에 적용한다. [현재 시작 순서](../my_arm_with_ball_application/scripts/keyboard_joint_teleop.py#L132-L148)
- Gazebo의 기존 그리퍼 `JointPositionController` 두 개와 명령 브리지 두 개는 새 명령 경로를 연결하는 같은 변경에서 제거한다. 동일 관절에 제어 명령이 중복 적용되지 않게 한다. [기존 Gazebo 플러그인](../my_arm_with_ball_description/models/ur5_rg2/model.sdf#L592-L610), [명령 브리지](../my_arm_with_ball_bringup/config/bridge.yaml#L6-L15)

공식 Jazzy 문서에서 `ForwardCommandController`의 `joints`, `interface_name: position`, `~/commands`의 `Float64MultiArray` 구성을 확인했다. 현재 모델의 두 손가락 관절은 mimic 관절이 아니라 각각 독립된 revolute 관절이다. [컨트롤러 문서](https://control.ros.org/jazzy/doc/ros2_controllers/forward_command_controller/doc/userdoc.html), [관절 정의](../my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro#L134-L144)

위 선택은 기존 팔 경로와 같은 메시지 형식 및 컨트롤러를 쓰는 최소 변경안이다. 손가락의 최종 각도와 대칭 동작은 보존 대상으로 삼고, 기존 Gazebo 플러그인의 과도 응답과 정확히 같은 속도나 궤적은 완료 조건으로 삼지 않는다. `gz_ros2_control`의 위치 명령은 위치 오차에 비례하는 관절 속도를 적용하므로 전환 후 응답 속도가 달라질 수 있다. 흔들림이나 큰 편차가 확인되면 실측 결과를 바탕으로 조정을 판단한다. [Gazebo 연동 문서](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html#notes-on-the-command-interface)

## 작업 순서

1. **변경 전 기준 확인:** 현재 `o → c → x`에서 두 손가락의 `/joint_states` 위치, 방향과 팔 키 입력 결과를 기록한다. 헤드리스 검증에서는 관절값을, 화면 검증이 가능하면 대칭 움직임을 확인한다. [기존 4단계 기준](ros2_control적용계획.md#4단계-통합-검증과-실습-정리)
2. **모델 제어 인터페이스:** `my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro`의 두 손가락 관절에 `position` 명령 인터페이스를 추가하고 상태 인터페이스는 유지한다. `my_arm_with_ball_description/test/test_model_consistency.py`의 명령 인터페이스 기대값을 8개 관절 모두 `position`으로 고친다. 관절 이름, 순서, 제한값과 SDF/URDF 기구학 일치 검사는 유지한다. [현 테스트](../my_arm_with_ball_description/test/test_model_consistency.py#L81-L95)
3. **컨트롤러 및 시작 순서:** `my_arm_with_ball_bringup/config/ros2_controllers.yaml`에 두 손가락 관절 순서와 `position` 인터페이스를 가진 `gripper_position_controller`를 정의한다. `my_arm_with_ball_bringup/launch/simulation.launch.py`에서 broadcaster 성공 후 팔 컨트롤러를 활성화하고, 팔 컨트롤러 성공 후 그리퍼 컨트롤러를 활성화한다. 실패 시 다음 spawner를 시작하지 않고 원인이 로그에 남게 한다. 기존 `forward_command_controller` 의존성은 재사용한다. [현재 활성화 절차](../my_arm_with_ball_bringup/launch/simulation.launch.py#L76-L126), [패키지 의존성](../my_arm_with_ball_bringup/package.xml#L12-L19)
4. **기존 제어 제거:** `my_arm_with_ball_description/models/ur5_rg2/model.sdf`의 그리퍼 `JointPositionController` 두 개와 `my_arm_with_ball_bringup/config/bridge.yaml`의 그리퍼 `cmd_pos` 매핑 두 개를 제거한다. `/clock` 브리지는 유지한다. 남은 관련 참조가 없는지 검색한다. [기존 플러그인](../my_arm_with_ball_description/models/ur5_rg2/model.sdf#L595-L610), [브리지 설정](../my_arm_with_ball_bringup/config/bridge.yaml#L1-L15)
5. **키보드 전환:** `my_arm_with_ball_application/scripts/keyboard_joint_teleop.py`의 관절별 `Float64` publisher 두 개를 그리퍼 컨트롤러용 `Float64MultiArray` publisher 하나로 바꾼다. `publish_gripper`, `o/c/x`, 시작 초기 명령을 두 값 배열로 연결한다. 기존 `wait_for_arm_controller`를 두 컨트롤러의 활성 상태를 확인하는 대기로 확장한다. 팔 목표값, 키 선택, 제한값, 재설정 범위는 보존한다. [현재 발행 및 대기](../my_arm_with_ball_application/scripts/keyboard_joint_teleop.py#L41-L100), [키 처리](../my_arm_with_ball_application/scripts/keyboard_joint_teleop.py#L160-L182)
6. **테스트와 실행 안내:** `my_arm_with_ball_application/test/test_keyboard_joint_teleop.py`에서 그리퍼 명령의 배열 길이·순서·값, `x`의 팔·그리퍼 재설정, 두 컨트롤러의 활성화 대기 및 시간 초과를 검증한다. 실행 안내는 실제 launch 인자와 별도 터미널의 키보드 실행 방식에 맞게 정리한다. 검증이 끝난 항목만 원 계획의 5단계 체크박스에 반영한다. [현 키보드 테스트](../my_arm_with_ball_application/test/test_keyboard_joint_teleop.py#L13-L79), [기존 실행 기록](ros2_control적용계획.md#4단계-통합-검증과-실습-정리)

## 검증 및 완료 조건

1. ROS 2 Jazzy 환경을 source한 뒤 네 패키지를 `colcon build --packages-select my_arm_with_ball_description my_arm_with_ball_gazebo my_arm_with_ball_bringup my_arm_with_ball_application --symlink-install`로 빌드한다. 같은 네 패키지의 `colcon test`와 `colcon test-result --verbose`를 확인한다. 모델 일치 테스트와 키보드 단위 테스트가 통과해야 한다.
2. `ros2 launch my_arm_with_ball_bringup simulation.launch.py headless:=true rviz:=false`를 실행한다. `ros2 control list_controllers`에서 `joint_state_broadcaster`, `arm_position_controller`, `gripper_position_controller`가 모두 `active`이고, `ros2 control list_hardware_interfaces`에서 그리퍼 두 관절의 `position` 명령 인터페이스가 제공·점유되는지 확인한다.
3. 별도 터미널에서 `ros2 run my_arm_with_ball_application keyboard_joint_teleop`을 실행한다. 시작, `o`, `c`, `x` 입력마다 `/gripper_position_controller/commands`에 두 값이 순서대로 발행되는지 확인한다. `/joint_states`에서 두 손가락이 `o` 뒤 각각 약 `1.18` rad, `c`와 `x` 뒤 각각 약 `0` rad로 수렴하는지 확인한다. 실제 상태는 명령 토픽과 구별해 판정한다.
4. `1~6`, `a/d`, `r`, `x`가 기존 팔 제어를 유지하고, `ros2 topic info /joint_states -v`에서 상태 발행자가 하나인지 확인한다. 기존 그리퍼 `/rg2_finger_joint1/cmd_pos`, `/rg2_finger_joint2/cmd_pos` 경로와 Gazebo 그리퍼 제어 플러그인이 활성 경로에 남아 있지 않아야 한다.
5. 시뮬레이션 재시작 후 컨트롤러 3개의 활성화, 초기 명령, `o/c`와 팔 명령을 다시 확인한다. 키보드 노드를 시뮬레이션보다 먼저 시작해도 두 컨트롤러 활성화 후 초기 목표가 전달되는지 확인한다. 활성화되지 않는 경우에는 제한 시간 내 오류와 비정상 종료를 확인한다.

완료 판정: 위 검증을 통과하고, 두 손가락의 목표각과 실제 움직임 및 팔의 기존 동작이 유지되며, 그리퍼를 제어하는 경로가 ros2_control 하나로 정리되었을 때 5단계를 완료로 기록한다. 화면으로 확인하지 못한 외형·충돌 항목이나 환경 때문에 실행하지 못한 검사는 결과와 한계를 별도로 기록한다.

## 주요 위험과 대응

- **그리퍼 명령 초기 유실:** 컨트롤러 활성화 확인 뒤 초기 배열을 발행하고, 시뮬레이션보다 키보드를 먼저 시작하는 검증을 포함한다.
- **이중 제어:** 모델의 기존 플러그인과 브리지 두 쌍을 함께 제거하고 설정 검색 및 실제 컨트롤러 상태를 확인한다.
- **동작 차이:** 기존 Gazebo 위치 플러그인의 게인과 ros2_control 위치 명령의 응답이 다를 수 있다. 우선 `0 ↔ 1.18` rad 도달, 양쪽 대칭, 진동 여부를 관측하고 필요할 때만 조정 범위를 결정한다.

## 실행 결과

- 계획한 별도 `gripper_position_controller`와 두 손가락 `position` 명령 인터페이스를 연결했다. 기존 Gazebo 그리퍼 위치 제어 플러그인 두 개와 그리퍼 명령 브리지 두 개를 제거했다. 키보드는 두 컨트롤러의 활성화를 기다린 뒤 팔 여섯 값과 그리퍼 두 값의 초기 명령을 보낸다.
- 첫 실행에서 `/gripper_position_controller/commands`에 `[1.18, 1.18]`이 전달되고 두 인터페이스가 `claimed`였지만, 오른쪽 손가락은 0 rad 하한에 머물렀다. 설치 환경은 Gazebo Sim `8.15.0`과 `ros-jazzy-gz-ros2-control` `1.2.20`이었다. 하한 `0.0`, 기본 게인 `0.1`에서 두 손가락의 측정값은 `[1.18, 0]` rad였다. 게인만 `0.01`로 낮춰도 오른쪽은 0 rad였다. 오른쪽 물리 하한만 `-0.001` rad로 낮추자 기본 게인에서 두 손가락 모두 `1.18` rad에 도달했고, `c → o → x` 왕복에서도 `[0, 0] → [1.18, 1.18] → [0, 0]` rad가 재현됐다. SDF와 Xacro 하한을 일치시켰다. 이는 관찰된 경계 정지를 피하는 물리 여유값이며, 라이브러리 내부 원인을 확정한 수정은 아니다. 키보드 명령 범위와 닫힘 목표 `0.0` rad는 유지한다. 게인 변경은 최종 설정에 포함하지 않았다.
- 네 패키지 `colcon build --symlink-install`, `colcon test`, `colcon test-result --verbose`가 통과했다(3개 테스트, 실패 0건). 모델 일치 검사에는 오른쪽 손가락 하한의 회귀 검사를 추가했다.
- 헤드리스 Gazebo에서 broadcaster와 팔·그리퍼 컨트롤러 세 개가 모두 `active`, 두 손가락의 `position` 명령 인터페이스가 `claimed`, `/joint_states` 발행자가 한 개임을 확인했다. 키보드 `o` 후 두 손가락은 각각 `1.18` rad, `c` 및 `x` 후 약 `0` rad였다. `1d`로 팔 첫 관절이 `0.05` rad로 움직였고 `x`로 `0` rad에 돌아왔다. 키보드를 시뮬레이션보다 먼저 실행한 경우와 시뮬레이션 재시작 후에도 명령을 확인했다. 이전 두 `cmd_pos` 토픽은 목록에 없었다.
- 헤드리스 실행이므로 화면상의 대칭 자세와 물체 접촉·충돌은 확인하지 않았다. 실제 공 파지는 이 단계의 완료 조건이 아니다.

재검증할 때는 같은 환경에서 시뮬레이션과 키보드 노드를 실행하고 `c → o → x`를 입력한다. 각 입력 뒤 `/joint_states`의 `rg2_finger_joint1`, `rg2_finger_joint2` 위치가 각각 약 `[0, 0]`, `[1.18, 1.18]`, `[0, 0]` rad로 수렴하는지 확인한다. 이 실행 검사는 물리 하한 회귀를 감지하기 위한 절차다.
