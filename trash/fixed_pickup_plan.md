# Fixed Pick-and-Place 구현 계획

## 목표

키보드에서 `s` 키를 한 번 입력하면 UR5+RG2 로봇팔이 다음 동작을 자동으로 수행한다.

1. 초기 자세(`HOME`)로 준비한다.
2. 그리퍼를 연다.
3. 공을 집는다.
4. 공을 바구니 안으로 옮겨 놓는다.
5. 로봇팔을 초기 자세로 복귀시킨다.

초기 구현은 물체 인식이나 동적 역기구학 대신, 현재 월드의 고정된 공과 바구니 위치에 맞춘 **joint-space 고정 웨이포인트** 방식으로 진행한다.

## 현재 프로젝트에서 활용할 인터페이스

- 로봇 모델: `ur5_rg2`
- 팔 관절 명령:
  - `/ur5_shoulder_pan_joint/cmd_pos`
  - `/ur5_shoulder_lift_joint/cmd_pos`
  - `/ur5_elbow_joint/cmd_pos`
  - `/ur5_wrist_1_joint/cmd_pos`
  - `/ur5_wrist_2_joint/cmd_pos`
  - `/ur5_wrist_3_joint/cmd_pos`
- 그리퍼 명령:
  - `/rg2_finger_joint1/cmd_pos`
  - `/rg2_finger_joint2/cmd_pos`
- 메시지 타입: `std_msgs/msg/Float64`
- 키보드 입력 노드: `my_arm_with_ball_application/scripts/keyboard_joint_teleop.py`
- 실행 구성: `my_arm_with_ball_bringup/launch/simulation.launch.py`

## 상태 머신

자동 동작은 다음 순서의 상태 머신으로 구현한다.

```text
HOME
  -> OPEN_GRIPPER
  -> APPROACH_BALL
  -> LOWER_TO_BALL
  -> CLOSE_GRIPPER
  -> LIFT_BALL
  -> MOVE_ABOVE_BASKET
  -> LOWER_INTO_BASKET
  -> OPEN_GRIPPER
  -> RETREAT
  -> RETURN_HOME
  -> DONE
```

### 상태별 의미

| 상태 | 동작 |
|---|---|
| `HOME` | 시작 시 로봇의 기준 관절 자세. 완료 후에도 이 자세로 돌아온다. |
| `OPEN_GRIPPER` | 공에 접근하기 전에 RG2 손가락을 완전히 연다. |
| `APPROACH_BALL` | 공과 충돌하지 않도록 공 위쪽의 접근 자세로 이동한다. |
| `LOWER_TO_BALL` | 그리퍼가 공을 감쌀 수 있는 높이까지 천천히 내려간다. |
| `CLOSE_GRIPPER` | 두 손가락을 닫아 공을 잡는다. 닫힌 뒤 안정화 대기 시간을 둔다. |
| `LIFT_BALL` | 공을 바닥에서 들어 올려 운반 높이까지 올린다. |
| `MOVE_ABOVE_BASKET` | 공을 충분히 높이 든 상태로 바구니 위쪽으로 이동한다. |
| `LOWER_INTO_BASKET` | 공을 바구니 내부의 놓기 위치까지 내린다. |
| `OPEN_GRIPPER` | 그리퍼를 열어 공을 바구니에 놓는다. |
| `RETREAT` | 바구니와 충돌하지 않도록 위쪽으로 후퇴한다. |
| `RETURN_HOME` | 시작 시 기준 자세 또는 사전에 정의한 홈 자세로 복귀한다. |
| `DONE` | 동작 완료를 로그로 알리고 다음 시작 입력을 대기한다. |

## 구현 단계

### 1. 수동 조작으로 웨이포인트 측정

현재 키보드 조작 노드를 사용하여 다음 자세의 관절값을 찾고 기록한다.

- `HOME`
- `APPROACH_BALL`
- `LOWER_TO_BALL`
- `LIFT_BALL`
- `MOVE_ABOVE_BASKET`
- `LOWER_INTO_BASKET`
- `RETREAT`

`OPEN_GRIPPER`와 `CLOSE_GRIPPER`는 현재 RG2 명령 범위에 맞춰 별도의 그리퍼 목표값으로 기록한다.

공의 월드 위치는 현재 SDF에서 대략 `(0.55, -0.25, 0.0325)`이고, 바구니의 위치는 대략 `(0.75, 0.25, 0.0)`이지만, 실제 모델의 충돌 형상과 그리퍼 높이를 기준으로 웨이포인트를 조정한다.

### 2. 자동 제어 노드 추가

`my_arm_with_ball_application`에 별도의 `pick_place_controller.py` 노드를 추가한다.

이 노드는 다음을 담당한다.

- 팔 관절 및 그리퍼 publisher 생성
- 웨이포인트별 관절 목표값 발행
- 상태 전환과 단계별 대기
- 실행 중 중복 시작 방지
- 완료 및 실패 로그 출력
- 필요 시 동작 중지 처리

각 상태는 ROS timer 기반으로 실행하여 블로킹 `sleep()`에 의존하지 않는다. 각 웨이포인트로 이동할 때는 충분한 안정화 시간을 두고 다음 상태로 전환한다.

### 3. `s` 키로 실행 시작

키보드 입력 처리에 `s` 키를 추가한다.

권장 구조는 다음과 같다.

```text
keyboard_joint_teleop.py
  -- s 입력
       -> /pick_place/start 서비스 호출
             -> pick_place_controller.py가 상태 머신 시작
```

서비스는 `std_srvs/srv/Trigger`를 사용한다.

- 실행 중이면 새 실행을 거부한다.
- 대기 상태이면 시퀀스를 시작한다.
- 호출 결과로 시작 가능 여부를 반환한다.

기존 수동 조작 기능은 유지한다.

### 4. 그리퍼와 공의 집기 방식 검증

첫 번째 시도에서는 기존 RG2 손가락 관절의 물리 접촉과 마찰을 사용한다.

검증 순서:

1. `OPEN_GRIPPER`에서 공에 접근한다.
2. `CLOSE_GRIPPER` 후 공이 손가락 사이에 유지되는지 확인한다.
3. `LIFT_BALL`에서 공이 함께 올라오는지 확인한다.
4. 바구니 위에서 `OPEN_GRIPPER` 후 공이 바구니 안에 떨어지는지 확인한다.

물리 접촉 방식이 불안정하면 `my_arm_with_ball_gazebo`의 Gazebo system plugin을 확장하여, 집기 시점에 공과 그리퍼를 attach하고 놓기 시점에 detach하는 보조 방식을 검토한다. 이 방식은 초기 시연 안정성을 높이는 대안이며, 물리 접촉 모델 자체를 대체할지 여부는 테스트 결과로 결정한다.

### 5. 검증용 상태 연결

자동 동작의 성공 여부를 확인하기 위해 다음 정보를 단계적으로 bridge한다.

- 로봇 joint state
- 공 pose
- 그리퍼 또는 end-effector pose
- 필요 시 접촉 상태

초기에는 로그와 Gazebo 화면으로 검증하고, 이후 공이 실제로 들어 올려졌는지와 바구니 안에 놓였는지를 자동 판정한다.

## 안전 및 오류 처리

- `s`를 반복 입력해도 실행 중인 시퀀스를 중복 시작하지 않는다.
- 각 관절 목표값은 모델의 허용 범위 안에서만 사용한다.
- 공을 잡지 못한 경우 바구니로 이동하지 않고 중단하거나 재시도할 수 있도록 상태를 분리한다.
- 동작 중 중지 입력을 추가할 수 있도록 상태 머신을 취소 가능한 구조로 만든다.
- 완료 후에는 `RETURN_HOME`을 거쳐야만 `DONE` 상태가 된다.

## 검증 순서

1. 기존 수동 키보드 제어가 계속 동작하는지 확인한다.
2. 팔 관절 웨이포인트만 실행하여 충돌 여부를 확인한다.
3. `OPEN_GRIPPER`와 `CLOSE_GRIPPER`의 손가락 동작을 확인한다.
4. `s` 입력으로 전체 상태 머신을 시작한다.
5. 공을 집은 채 이동하는지 확인한다.
6. 공이 바구니 안에 놓이는지 확인한다.
7. 로봇팔이 `HOME`으로 복귀하는지 확인한다.
8. 필요하면 Gazebo 버전에 맞춰 접촉 파라미터 또는 attach/detach 방식을 조정한다.

## 구현 후 실행 형태

최종적으로는 다음과 같은 방식으로 실행하는 것을 목표로 한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py keyboard:=true
```

실행된 인터랙티브 터미널에서 `s` 키를 입력하면 다음 순서가 시작된다.

```text
HOME
-> OPEN_GRIPPER
-> APPROACH_BALL
-> LOWER_TO_BALL
-> CLOSE_GRIPPER
-> LIFT_BALL
-> MOVE_ABOVE_BASKET
-> LOWER_INTO_BASKET
-> OPEN_GRIPPER
-> RETREAT
-> RETURN_HOME
-> DONE
```

이 문서 단계에서는 계획만 작성하며, 실제 코드와 SDF는 수정하지 않는다.
