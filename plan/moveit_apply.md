# MoveIt 2 적용 단계별 구현 계획

## 1. 목표

UR5 + RG2가 Gazebo Sim에서 월드에 고정 배치된 공을 집어 바구니 중심 위로 옮겨 놓고, 시작 관절 자세로 돌아오는 작업을 구현한다. MoveIt 2가 전체 팔 동작을 계획하고, RViz에서 계획 경로를 확인하며, 실제 실행은 Gazebo의 로봇과 그리퍼에서 수행한다.

최종 작업 흐름:

```text
시작 상태 확인
  → 공에 접근
  → 그리퍼 닫기
  → 접촉·개방각 조건 확인
  → 공을 그리퍼에 부착하고 동반 이동 확인
  → 바구니 중심 위로 이동
  → 공 분리
  → 공이 안정된 뒤 바구니 안에 있는지 확인
  → 시작 관절 자세로 복귀
```

어느 구간에서든 계획, 실행, 집기 또는 놓기 판정이 실패하면 작업을 중단하고 원인과 실패 단계를 ROS 로그에 남긴다. 자동 재시도는 하지 않는다.

## 2. 합의된 동작 및 성공 조건

- 공과 바구니는 월드 기준의 고정 위치를 사용한다.
- 집기 성공은 다음 조건을 모두 만족할 때로 정의한다.
  1. 그리퍼와 공의 접촉이 검출된다.
  2. 그리퍼 전체 개방 각도가 5°를 초과한 상태에서 닫기 동작이 완료된다.
  3. 공이 그리퍼에 부착되고, 이후 그리퍼와 함께 움직이는 것이 확인된다.
- 공은 바구니 중심 위에서 분리한다.
- 놓기 성공은 공이 낙하 후 안정된 시점에 바구니 입구 안에 있는 것으로 판정한다.
- 정상 완료 후 로봇 팔은 작업 전 시작 관절 자세로 복귀한다.
- 사용자는 RViz에서 계획을 확인하고, Gazebo에서 실행 결과를 확인한다.

## 3. 현재 저장소 기준선과 선행 조사 항목

저장소에서 확인한 현재 구성:

- `my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro`에 UR5, RG2, `tool0`, 팔 6축 및 손가락 관절이 정의되어 있다.
- `my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf`가 Gazebo Sim 월드이며, 현재 공은 `(0.55, -0.25, 0.0325)`, 바구니는 `(0.75, 0.25, 0.000022)` 위치에 배치되어 있다.
- `my_arm_with_ball_bringup/launch/simulation.launch.py`가 Gazebo, `ros_gz_bridge`, `robot_state_publisher`, RViz와 ros2_control 컨트롤러를 실행한다.
- 현재 arm/gripper 컨트롤러는 `ForwardCommandController`이며, MoveIt이 기대하는 FollowJointTrajectory 액션 인터페이스는 아직 설정되어 있지 않다.
- 현재 RViz 실행은 URDF 표시용 설정이며 MoveIt MotionPlanning 플러그인 구성은 확인되지 않았다.
- 현재 저장소에서 MoveIt 패키지 설정, Gazebo 접촉 감지, 공 부착/분리를 수행하는 기능은 확인되지 않았다.
- 기존 좌표 정의에서 공·바구니 위치의 기준은 Xacro가 아니라 월드 SDF다. 로봇의 링크·관절 정의와 고정 베이스 변환은 Xacro를 기준으로 한다.

구현 전 ROS 배포판, Gazebo 배포판, MoveIt 2 및 `ros_gz` 설치 버전의 호환성을 확인한다. 저장소 README도 ROS/Gazebo 조합을 현재 개발 환경 기준으로 설명하므로, 확인 결과를 기반으로 의존 패키지와 실행 구성을 확정한다.

## 4. 단계별 구현

### 단계 1 — 버전 호환성과 로봇 모델 기준 확정

**작업**

- [x] 사용 중인 ROS 2 배포판, Gazebo Sim, MoveIt 2, `ros_gz` 버전을 기록하고 공식 호환 조합을 확인한다. Ubuntu 24.04.5 / ROS 2 Jazzy / Gazebo Harmonic (`gz-sim` 8.15.0) / MoveIt 2 2.12.4 / `ros_gz` 1.0.24 / `gz_ros2_control` 1.2.20을 확인했다.
- [x] Xacro에서 planning group 후보 팔 6축, gripper 2축, `base_link`, `tool0`, `rg2_hand`를 확인한다. 실제 EEF tip은 `rg2_hand`로 정하고 `tool0`는 보조 프레임으로 둔다.
- [x] 그룹과 시작 자세 후보를 정한다: `ur5` 6축, `gripper` 2축, 명목 `home`은 8개 관절 모두 0 rad다. URDF root가 `world`이고 고정 변환으로 `base_link`에 연결되므로 planning frame은 `world`, 별도 virtual joint는 두지 않는다.
- [x] URDF/Xacro의 충돌 형상과 관절 한계를 확인한다. MoveIt Setup Assistant에서 robot model 및 package URI mesh를 읽고, SDF/Xacro consistency 검사 4개 항목이 통과했다.
- [x] SDF 월드의 공·바구니 pose를 월드 기준 목표로 기록하고, 공 반지름과 바구니 mesh AABB를 조사했다. 바구니 유효 내부 경계는 미확정으로 단계 2/6에 넘긴다.
- [x] SRDF 파일 작성 및 self-collision matrix 생성·검토를 2단계에서 완료했다. 상세 판정은 [`m_2_plan.md`](m_2_plan.md)에 기록했다.

**완료 조건**

- [x] 지원되는 의존성 조합과 설치 방법을 확정한다. 현재 설치된 Jazzy 바이너리와 Jazzy/Harmonic 공식 지원 조합을 유지한다.
- [x] MoveIt Setup Assistant가 Xacro URDF와 collision mesh를 로드하고, 시뮬레이션에서 팔 시작 관절 상태가 명목 0 rad임을 확인한다. 작업 목표는 월드 frame 좌표로 표현할 수 있게 정리했다.
- [x] 계획 프레임 `world`, 로봇 베이스 `base_link`, end-effector 기준 `rg2_hand`와 보조 `tool0` 사이 관계를 확인한다.
- [x] SRDF 그룹/자세와 충돌행렬을 생성·검토했다. 바구니는 원본 내부 표면을 포함한 mesh를 충돌 모델로 사용하며, containment 판정에 필요한 수치 기준은 단계 6에서 정한다.

### 단계 2 — MoveIt 설정 패키지와 RViz 계획 검증

**작업**

- [x] 전용 MoveIt 설정 패키지를 추가하고 URDF/Xacro, SRDF, kinematics, joint limits, OMPL, RViz 설정을 구성한다.
- [x] Gazebo 실행과 별도로 MoveIt/RViz 계획 검증을 실행할 수 있게 한다.
- [x] 팔 시작 자세에서 공 접근, lift, 바구니 위 목표까지 충돌 없는 경로를 계획하고 궤적 중간 상태를 검사한다.
- [x] 공과 바구니를 MoveIt Planning Scene의 충돌 물체로 등록한다. Gazebo 부착/분리와 attached-object 갱신은 4단계에서 구현한다.

**완료 조건**

- [x] RViz MotionPlanning 및 MoveGroup 계획 서비스가 UR5 그룹과 목표를 처리하고 유효한 경로를 만든다.
- [x] 검증한 계획 경로가 로봇 자기 충돌, 공, 바구니와 충돌하지 않는다. 공/바구니에 의도적으로 겹치는 음성 대조 상태는 충돌로 판정됐다.
- [x] 현재 SDF 위치 기준의 공 접근, lift, 바구니 위 작업 포즈가 도달 가능함을 확인했다.

단계 2 구현·검증 기록과 알려진 제한은 [`m_2_plan.md`](m_2_plan.md)에 있다. 계획 전용 launch에서 Gazebo 궤적 실행 action은 활성화하지 않았다.

### 단계 3 — Gazebo ros2_control과 MoveIt 실행 인터페이스 연결

상태: **완료 (2026-10-13).** 실제 구현 및 헤드리스 Gazebo 실행 결과는 [`m_3_plan.md`](m_3_plan.md)에 기록했다.

**작업**

- [x] 팔 `ForwardCommandController`를 `joint_trajectory_controller`로 교체하고 `FollowJointTrajectory` 액션을 사용한다. 변환 어댑터 방식은 채택하지 않으며 기존 팔 위치 컨트롤러는 제거한다.
- [x] 팔 trajectory 컨트롤러를 MoveIt에 연결하고, 그리퍼는 기존 위치 명령 경로를 유지한다. 그리퍼의 MoveIt 실행 연결은 후속 3단계 보강 항목에서 별도로 수행한다.
- [x] MoveIt trajectory execution을 통합 Gazebo launch에서 허용하고, 계획 전용 MoveIt 데모는 실제 실행을 계속 비활성화한다.
- [x] 기존 키보드 teleop을 실제 `/joint_states` 기반의 짧은 상대 궤적 액션으로 전환하고, 컨트롤러 활성 순서를 정리한다.

**완료 조건**

- [x] MoveIt이 계획한 짧은 팔 궤적을 Gazebo 로봇이 추종한다. GUI/RViz 화면 검증은 헤드리스 실행에 포함하지 않았다.
- [x] MoveIt 상태 모니터 및 `/joint_states`에서 실제 실행 결과를 확인했다.
- [x] controller 비활성, 액션 거부, 실행 timeout 등 오류 유형을 구분해 한국어·영어 로그로 보고하고, 액션 실패 후 키보드 노드는 계속 실행하도록 구현했다.

**검증 결과**

- 전체 workspace `colcon build --symlink-install` 및 `colcon test`/`colcon test-result --verbose` 통과(3개 테스트, 실패 0).
- `simulation.launch.py headless:=true rviz:=false` 실행에서 `joint_state_broadcaster`, `arm_trajectory_controller`, `gripper_position_controller`가 모두 `active`였다. 팔 6개 position 명령 인터페이스는 trajectory controller가 점유했고, 기존 `arm_position_controller`는 등록되지 않았다. `/joint_states` 발행자는 하나였다.
- 키보드 동작 경로를 실제 action/controller/Gazebo에 연결해 `+0.05 rad` 목표를 수행했다. 측정 상태는 시작 약 `0.000 rad`에서 `0.0450 rad`로 변했고 목표 `0.050 rad`와 최대 `0.0050 rad` 차이로 허용 오차 `0.01 rad` 이내였다.
- MoveIt `MoveGroup` action으로 충돌 없는 `shoulder_pan_joint` 소량 이동을 계획·실행했다. `MoveItErrorCodes.SUCCESS`와 action 성공을 확인했고 관절 상태가 목표 허용 오차 이내로 움직였다. 별도 실행 시험에서는 MoveIt `allowed_start_tolerance=0.02 rad` 초과의 시작 궤적을 거부했다.
- 독립 `demo.launch.py use_rviz:=false`에서 `allow_trajectory_execution=False`이고 `/execute_trajectory` action이 제공되지 않아 계획 전용 경계를 유지함을 확인했다.
- 키보드 조작의 충돌 검사는 MoveIt을 거치지 않는 직접 controller action이므로 소량의 안전한 관절 동작으로만 검증했다. RViz GUI 시각 검증 및 공 집기·그리퍼 MoveIt 연결은 이번 단계 범위가 아니다.

### 단계 3 후속 — MoveIt 그리퍼 실행 연결

팔 궤적 실행을 검증한 뒤, 접촉 감지나 공 부착/분리 기능에 앞서 RG2 그리퍼 자체를 MoveIt에서 실행할 수 있도록 연결한다. 이 항목은 팔 중심 3단계와 기존 접촉·부착 4단계 사이의 별도 선행 작업이다. 기존 단계 번호는 유지한다.

상태: **완료 (2026-10-14).** 설계와 검증 결과는 [`m_3_후속_설계.md`](m_3_후속_설계.md)에 기록했다. 팔·그리퍼 keyboard action은 상호 배타적으로 실행한다.

**작업**

- 단계 2에서 만든 SRDF의 `gripper` 그룹과 두 finger joint 정의를 재사용하고, 실제 계획 모델에서 그룹 관절 이름·순서·한계를 다시 대조한다. 불필요한 새 그룹이나 joint mimic 관계를 추가하지 않는다.
- Gazebo ros2_control의 `gripper_position_controller`를 두 finger joint용 `joint_trajectory_controller`로 전환해 `FollowJointTrajectory` 액션 서버를 제공한다. 두 finger joint의 현재 독립 position command/state 인터페이스를 사용한다.
- MoveIt controller manager 설정에 gripper trajectory controller의 이름, action endpoint, `gripper` 그룹의 두 관절 매핑을 추가한다. 팔 및 그리퍼 컨트롤러가 각각 자기 관절만 점유하고 MoveIt이 두 컨트롤러를 올바른 그룹에 선택하는지 확인한다.
- `simulation.launch.py`에서 joint state broadcaster 성공 후 팔 및 그리퍼 trajectory controller를 활성화하고 MoveIt 통합 실행을 유지한다. 같은 finger joint를 점유하는 기존 `gripper_position_controller`를 동시에 실행하지 않는다.
- 키보드 `o/c` 조작을 기존 gripper position topic에서 gripper trajectory action으로 전환한다. 각 동작은 현재 finger joint 상태를 시작점으로 사용하고, 액션 결과와 `/joint_states`의 실제 목표 도달을 확인한다. 팔 키 동작과 기존 입력 중첩·실패 규칙을 일관되게 적용한다.
- MoveIt의 `gripper` 그룹에서 열림/닫힘 joint 목표를 각각 계획·실행하고 Gazebo 상태와 비교한다. 목표 각도는 기존 `o=1.18 rad`, `c=0 rad` 동작을 기준으로 하며, 두 finger joint의 방향과 상태값을 실측 확인한다.
- 액션 거부, timeout, 취소, 상태 미도달 시 원인이 명확히 보고되며, 공 집기 성공이나 물리적 파지 성능을 이 단계의 결과로 간주하지 않는다.

**완료 조건**

- MoveIt controller manager가 `gripper` 그룹을 gripper trajectory controller의 `FollowJointTrajectory` 액션에 연결하고, MoveIt에서 그리퍼 궤적을 Gazebo에 실행한다.
- 두 finger joint가 지정된 열림/닫힘 목표로 이동하고, action result와 `/joint_states` 결과가 설정된 허용 오차 안에서 일치한다.
- 팔과 그리퍼 trajectory controller가 동시에 활성화되어도 서로 다른 joint command interface만 점유한다. 기존 gripper position controller는 같은 finger joint의 제어 경로로 남아 있지 않는다.
- 키보드 `o/c`가 기존 기능 의미를 보존하면서 action 경로로 동작하고, action 실패·timeout이 로그에서 구별된다.
- Planning Scene에서 그리퍼 상태와 Gazebo `/joint_states`가 동기화되며, standalone 계획 전용 실행은 계속 실제 trajectory 실행을 비활성화한다.
- 접촉 감지, 공 부착/분리, 파지 성공 판정은 아직 구현하지 않았고 다음 단계 범위로 명확히 남긴다.

### 단계 4 — 그리퍼 상태 및 Gazebo 접촉·부착 동작

**상태: 구현 및 헤드리스 통합 검증 완료 (2026-10-14).** 구현·시험 범위와 실행 방법은 [`4단계_설계.md`](4단계_설계.md) 및 [`control_validation_checklist.md`](../docs/control_validation_checklist.md)를 참조한다. RViz 화면의 수동 시각 확인은 별도 미실행이다.

**작업**

- RG2 전체 개방 각도를 두 finger joint 상태로부터 일관되게 계산하고, 각도 기준 단위 및 방향을 확인한다. 임계값 5°는 라디안 기반 관절 상태와 비교할 때 변환을 명시한다.
- Gazebo에서 그리퍼-공 접촉을 관찰할 수단(접촉 센서 또는 적절한 시뮬레이터 상태 인터페이스)을 정하고, 실제로 신뢰성 있게 읽히는지 검증한다.
- 공과 그리퍼의 접촉 및 합의된 개방 각도 조건이 모두 맞은 경우에만 공을 그리퍼 링크에 부착한다.
- 부착/분리 기능은 Gazebo 측 동작과 MoveIt Planning Scene의 attached collision object 상태를 함께 갱신하도록 구성한다.
- 분리 후 공이 동역학에 따라 낙하하도록 하고, 바구니 충돌 형상과 입구 형상을 확인한다.

**완료 조건**

- 빈 그리퍼 닫기와 공을 잡은 그리퍼 닫기를 구분한다.
- 조건을 만족하지 않으면 공을 부착하지 않고 집기 실패를 보고한다.
- 부착된 공이 팔 이동 동안 그리퍼와 함께 움직이고, 분리 요청 뒤에는 독립 물체로 낙하한다.
- MoveIt 계획 장면과 Gazebo의 공 상태가 각 전환 직후 일치한다.

실제 시험에서는 빈손 부착 거부, 접촉 후 MoveIt 닫기 동작 취소·종료 확인, 5° 초과 상태에서 부착, 동반 상승, 분리·낙하, 낙하 후 Planning Scene 위치 갱신까지 확인했다. 접촉 직후 제어 취소로 접촉이 사라질 수 있어 Gazebo는 유효 접촉 증거를 3초만 보존하며, 시간이 지나거나 개방각 조건이 깨지면 부착을 거부한다. 바구니 이송은 단계 5 범위다.

### 단계 5 — 전체 Pick-and-Place 작업 노드

**작업**

- 하나의 작업 노드가 `접근 → 닫기/검증 → 부착 → 들어올리기 → 바구니 위 이동 → 분리 → 놓기 검증 → 시작 자세 복귀` 상태 흐름을 실행한다.
- 각 구간은 MoveIt 계획/실행 결과를 확인한 뒤 다음 상태로 넘어간다.
- 시작 자세, 공 접근 자세, 안전한 lift 높이, 바구니 중심 위 놓기 자세는 기존 모델/월드 좌표와 RViz 계획 검증을 바탕으로 설정값으로 관리한다.
- 작업 중단 시 현재 상태와 실패 원인을 기록하고, 성공으로 오인하지 않도록 한다. 합의대로 자동 재시도는 수행하지 않는다.
- 동시 실행 요청, 중복 작업 시작, 취소/종료 시 동작을 명시해 작업 상태가 꼬이지 않도록 한다.

**완료 조건**

- 단일 실행 요청으로 전체 작업이 끝나고 시작 관절 자세로 복귀한다.
- 각 상태 진입/완료 및 실패 단계가 로그에서 추적된다.
- 계획 실패, controller 실행 실패, 집기 실패, 놓기 실패가 각각 구분되어 중단된다.

### 단계 6 — 놓기 판정, 통합 검증 및 실행 절차

**작업**

- 공의 위치·속도로 낙하 안정 여부를 판정하고, 바구니 입구 안에 있는지를 월드 좌표에서 평가한다.
- 안정 판정 시간/속도 기준과 입구 내부 판정의 기하학적 정의를 설정값으로 두고, 현재 바구니 형상과 공 크기에 맞춰 결정한다.
- 성공/실패를 의도적으로 유발해 로그, 상태 전이, Planning Scene 동기화 및 재실행 가능성을 확인한다.
- Gazebo+MoveIt 통합 실행 launch, RViz 표시, 노드 실행 및 검증 순서를 문서화한다.

**완료 조건**

- 공이 안정된 후 입구 안에 남아 있을 때만 놓기 성공으로 판정한다.
- 공이 바구니 밖에 있거나 판정을 완료할 수 없으면 작업이 중단되고 원인이 기록된다.
- 정상 시나리오와 주요 실패 시나리오가 재현 가능하다.

## 5. 구현 원칙과 범위

- 이 계획은 고정된 SDF 작업 위치를 사용하는 시뮬레이션용 MoveIt 적용을 대상으로 한다. 비전 인식, 동적 물체 추적, 실물 로봇 제어는 범위에 포함하지 않는다.
- 접촉 후 공을 부착하는 방식은 작업 시퀀스 및 경로 실행 검증을 위한 시뮬레이션 처리다. RG2의 실제 파지력, 마찰, 미끄러짐은 검증하지 않는다.
- 공 부착/분리 및 접촉 감지는 MoveIt 자체가 Gazebo 물리 동작으로 자동 제공한다고 가정하지 않는다. Gazebo 측 구현과 Planning Scene 동기화가 필요하다.
- MoveIt 설정/컨트롤러 연결을 먼저 독립 검증하고, 마지막에 단일 작업 노드로 통합한다.
- 기존 `adr/Pick_and_Place_Implementation_Strategy.md`는 고정 Joint Waypoint에서 시작해 MoveIt으로 발전하는 기존 전략을 기록한다. 이 계획은 사용자가 선택한 MoveIt 기반 목표를 위한 구체화 문서이며, 기존 ADR의 구현 순서와 다른 점은 실제 착수 시 함께 정합성 검토한다.

## 6. 추후 구현 시 정할 세부 파라미터

아래 항목은 현재 대화에서 동작 의미를 합의했으며, 실제 모델과 통합 테스트를 바탕으로 수치 또는 구현 인터페이스를 정한다.

- 공이 안정됐다고 보는 속도 임계값과 연속 안정 시간
- 바구니 입구 안쪽 판정 기준(공 중심 기준 또는 공 형상 포함 여부)
- 접촉 상태를 가져오는 Gazebo 인터페이스와 부착/분리 서비스 또는 시스템 경계
- 그리퍼 전체 개방 각도를 산출하는 구체적 관절 상태 식과 5° 경계 비교 방식
- MoveIt 플래너, 계획 시도/timeout, 경로 스케일링 등 실행 설정
- 작업 노드의 ROS 인터페이스(예: action/service)와 사용자 실행 방법
