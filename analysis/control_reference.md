# Fixed Joint Waypoint 제어 기준

이 문서는 고정 joint waypoint를 만들 때 사용자가 알아야 할 현재 프로젝트의 제어 정보를 정리한 문서다. 현재 단계에서는 코드와 SDF를 변경하지 않고, 기존 모델과 토픽을 기준으로 웨이포인트를 조정한다.

## 1. 제어 대상

월드 파일 `my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf`는 다음 모델을 포함한다.

| 대상 | 모델/위치 |
|---|---|
| 로봇 | `model://ur5_rg2` |
| 로봇 기준 pose | `(0, 0, 0, 0, 0, 0)` |
| 공 | Gazebo Fuel의 `RoboCup SPL Ball` |
| 공의 월드 pose | `(0.55, -0.25, 0.0325, 0, 0, 0)` |
| 바구니 | Gazebo Fuel의 `Threshold_Basket_Natural_Finish_Fabric_Liner_Small` |
| 바구니의 월드 pose | `(0.75, 0.25, 0.000022, 0, 0, 0)` |

공과 바구니의 pose는 모델의 기준점이다. 실제 공 중심, 바구니 입구와 내부 바닥의 위치는 Fuel 모델의 형상에 따라 기준 pose와 다를 수 있으므로, waypoint 위치를 정할 때는 Gazebo 화면에서 실제 충돌 형상을 확인해야 한다.

로봇은 `ur5_rg2_joint_world` fixed joint로 `world`에 고정되어 있다. 따라서 고정 waypoint에서는 로봇 전체를 이동시키지 않고 6개의 회전 관절만 제어한다.

## 2. 팔 관절 이름과 명령 토픽

팔은 다음 6개 revolute joint를 사용한다. 아래 순서는 모든 waypoint에서 동일하게 유지해야 한다.

| 순서 | Joint 이름 | ROS 명령 토픽 | Gazebo 명령 토픽 |
|---:|---|---|---|
| 1 | `shoulder_pan_joint` | `/ur5_shoulder_pan_joint/cmd_pos` | `/model/ur5_rg2/joint/shoulder_pan_joint/0/cmd_pos` |
| 2 | `shoulder_lift_joint` | `/ur5_shoulder_lift_joint/cmd_pos` | `/model/ur5_rg2/joint/shoulder_lift_joint/0/cmd_pos` |
| 3 | `elbow_joint` | `/ur5_elbow_joint/cmd_pos` | `/model/ur5_rg2/joint/elbow_joint/0/cmd_pos` |
| 4 | `wrist_1_joint` | `/ur5_wrist_1_joint/cmd_pos` | `/model/ur5_rg2/joint/wrist_1_joint/0/cmd_pos` |
| 5 | `wrist_2_joint` | `/ur5_wrist_2_joint/cmd_pos` | `/model/ur5_rg2/joint/wrist_2_joint/0/cmd_pos` |
| 6 | `wrist_3_joint` | `/ur5_wrist_3_joint/cmd_pos` | `/model/ur5_rg2/joint/wrist_3_joint/0/cmd_pos` |

ROS 명령은 `std_msgs/msg/Float64`로 발행한다. `bridge.yaml`이 ROS 토픽을 Gazebo 토픽으로 `ROS_TO_GZ` 방향으로 연결한다. 따라서 waypoint 실행기는 우선 ROS 명령 토픽만 발행하면 된다.

각 waypoint는 6개 관절의 목표값을 모두 가져야 한다. 특정 관절을 조정하지 않는 상태라도 이전 값 또는 원하는 값을 명시적으로 포함한다.

## 3. 초기 자세와 관절 제한

### 초기 자세

현재 수동 키보드 노드가 사용하는 초기 목표값은 다음과 같다.

| Joint | 초기 목표값(rad) |
|---|---:|
| `shoulder_pan_joint` | `0.0` |
| `shoulder_lift_joint` | `-1.5708` |
| `elbow_joint` | `0.0` |
| `wrist_1_joint` | `-1.5708` |
| `wrist_2_joint` | `0.0` |
| `wrist_3_joint` | `0.0` |

이 값은 `keyboard_joint_teleop.py`의 `INITIAL_POSITIONS`에 정의된 애플리케이션 기준값이다. `HOME` waypoint의 초안으로 사용할 수 있지만, 실제 Gazebo 화면에서 로봇의 시작 자세가 기대한 자세인지 먼저 확인한다.

### SDF 관절 제한

현재 UR5 모델 SDF의 6개 팔 관절은 모두 다음 범위를 갖는다.

```text
-6.28319 rad <= joint position <= 6.28319 rad
```

관절별 effort와 velocity도 SDF에 정의되어 있지만, waypoint 작성 시 우선 지켜야 할 것은 위치 범위다. 자동 시퀀스의 목표값은 이 범위를 넘지 않아야 한다.

| Joint | Lower (rad) | Upper (rad) |
|---|---:|---:|
| `shoulder_pan_joint` | `-6.28319` | `6.28319` |
| `shoulder_lift_joint` | `-6.28319` | `6.28319` |
| `elbow_joint` | `-6.28319` | `6.28319` |
| `wrist_1_joint` | `-6.28319` | `6.28319` |
| `wrist_2_joint` | `-6.28319` | `6.28319` |
| `wrist_3_joint` | `-6.28319` | `6.28319` |

## 4. 그리퍼 명령값과 제어 방식

RG2 그리퍼는 두 개의 독립적인 revolute joint로 구성된다.

| Joint | ROS 명령 토픽 | SDF axis | 위치 범위 |
|---|---|---|---|
| `rg2_finger_joint1` | `/rg2_finger_joint1/cmd_pos` | `0 0 1` | `0.0` ~ `1.18` rad |
| `rg2_finger_joint2` | `/rg2_finger_joint2/cmd_pos` | `0 0 -1` | `0.0` ~ `1.18` rad |

두 관절에는 동일한 수치의 `Float64` 명령을 각각 발행한다. 현재 수동 조작 노드의 기준은 다음과 같다.

| 동작 | 명령값 |
|---|---:|
| `INITIAL_GRIPPER_POSITION` | `0.0` |
| `c` 키: 닫기 | `0.0` |
| `o` 키: 열기 | `1.18` |
| 허용 범위 | `0.0` ~ `1.18` |

현재 코드와 SDF의 값으로는 `0.0`을 닫힘, `1.18`을 열림으로 사용하는 것이 프로젝트의 기준이다. 다만 손가락의 실제 시각적 방향과 공을 잡을 때 필요한 간격은 Gazebo에서 반드시 확인한다. 특히 두 joint의 회전축 부호는 반대지만, 명령 위치값은 동일하게 발행하도록 모델과 현재 bridge가 구성되어 있다.

그리퍼의 Gazebo controller는 `ignition-gazebo-joint-position-controller-system`이며, 두 finger joint에도 위치 명령을 적용한다. 현재 제어는 힘 제어 또는 grasp action이 아니라 목표 joint position을 전달하는 방식이다. 따라서 닫힘 명령만으로 공이 항상 안정적으로 고정된다고 보장할 수 없으며, `CLOSE_GRIPPER` 후 `LIFT_BALL`에서 실제 접촉과 유지 여부를 시험해야 한다.

## 5. Joint position controller

UR5 모델의 각 팔 관절과 그리퍼 관절에는 Gazebo joint position controller plugin이 연결되어 있다.

```xml
<plugin
  filename="ignition-gazebo-joint-position-controller-system"
  name="ignition::gazebo::systems::JointPositionController">
  <joint_name>...</joint_name>
</plugin>
```

팔과 그리퍼는 모두 위치 목표를 받는다. 따라서 waypoint 실행의 기본 단위는 다음과 같다.

1. 6개 팔 joint에 목표 radian 값을 각각 발행한다.
2. 두 finger joint에 같은 그리퍼 목표값을 발행한다.
3. 로봇이 목표 자세에 안정화될 때까지 기다린다.
4. 다음 상태로 넘어간다.

현재 position command에는 도달 여부를 알려주는 별도 ROS 응답이 없다. 초기 조정 단계에서는 상태별 고정 대기 시간과 Gazebo 화면 확인을 사용한다. 이후 자동 판정이 필요하면 joint state와 model/link pose bridge를 추가해야 한다.

## 6. 상태별로 사용자가 확인할 정보

각 상태의 waypoint를 확정할 때 다음 정보를 6개 관절 목표값과 함께 기록한다.

| 항목 | 확인 내용 |
|---|---|
| 팔 자세 | 6개 joint 목표값(rad) |
| 그리퍼 | `0.0` 닫힘 또는 `1.18` 열림, 필요하면 중간값 |
| 대상 위치 | 공 또는 바구니에 대한 상대 위치 |
| 접근 높이 | 충돌을 피하는 여유 높이 |
| 안정화 시간 | 다음 상태 전에 기다릴 시간 |
| 성공 기준 | 공 위 정렬, 공을 들어 올림, 바구니 내부 투입 등 |

권장 상태 순서는 다음과 같다.

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

`OPEN_GRIPPER`는 같은 상태 이름이 두 번 사용되므로 데이터 구조에서는 의미를 구분할 수 있도록 `OPEN_GRIPPER_BEFORE_APPROACH`와 `OPEN_GRIPPER_IN_BASKET`처럼 표현하거나, 상태 머신의 서로 다른 전이 단계로 관리한다.

## 7. 아직 실행 확인이 필요한 사항

다음 내용은 파일 분석만으로 확정할 수 없고, Gazebo를 실행해 화면으로 확인해야 한다.

- 각 관절의 양수/음수 명령이 화면상 어느 방향으로 움직이는지
- 초기 목표값이 실제 `HOME` 자세로 적절한지
- `0.0`과 `1.18`에서 그리퍼가 실제로 닫히고 열리는지
- `APPROACH_BALL`에서 그리퍼 중심이 공의 수직 위에 위치하는지
- `LOWER_TO_BALL`에서 손가락이 공을 감싸는지
- `CLOSE_GRIPPER` 후 `LIFT_BALL`에서 공이 함께 움직이는지
- 바구니의 실제 입구와 내부 바닥 위치
- 목표 pose 도달에 필요한 상태별 안정화 시간

따라서 이 문서의 수치 중 관절 초기값과 제한값은 코드/SDF 기준으로 사용할 수 있지만, 공과 바구니에 맞는 최종 waypoint joint 값은 Gazebo에서 미세 조정해 확정해야 한다.

## 8. 사용자 확인용 실행 기준

수동 확인을 할 때는 다음 실행 구성을 사용한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
source install/setup.bash
ros2 launch my_arm_with_ball_bringup simulation.launch.py keyboard:=true
```

키보드 노드에서:

- `1`~`6`: 조정할 팔 joint 선택
- `a`: 선택 joint를 `-0.05` rad 이동
- `d`: 선택 joint를 `+0.05` rad 이동
- `o`: 그리퍼 열기 (`1.18`)
- `c`: 그리퍼 닫기 (`0.0`)
- `x`: 팔과 그리퍼를 초기 목표값으로 재설정
- `q`: 종료

이 수동 도구는 최종 자동 제어기가 아니다. 각 상태의 실제 joint 목표값을 찾고 기록하기 위한 기준 도구다.
