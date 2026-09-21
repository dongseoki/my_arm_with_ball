# Pick & Place 구현 전략 결정

## 1. 결정 사항

현재 `my_arm_with_ball_gazebo`의 Pick & Place 기능은 **고정 Joint Waypoint 기반으로 먼저 구현**한다.

이후 기능을 단계적으로 발전시켜 **IK → Trajectory / Motion Planning → TF2 → Vision → Feedback**을 적용한다.

즉, 처음부터 모든 로봇공학 기술을 적용하지 않고, **동작 성공을 위한 최소 시스템을 먼저 완성한 뒤 각 단계의 수동 개입을 로봇공학 기술로 대체하는 방식**을 선택한다.

---

## 2. 목표

키보드에서 `s`를 입력하면 다음 작업이 자동으로 수행되도록 한다.

```text
s 입력
  ↓
초기화
  ↓
그리퍼 OPEN
  ↓
HOME
  ↓
공 접근
  ↓
공 집기
  ↓
공 들어올리기
  ↓
바구니 위로 이동
  ↓
바구니에 공 놓기
  ↓
후퇴
  ↓
HOME 복귀
```

초기 버전에서는 공과 바구니의 위치를 고정하고, 각 단계의 Joint 값을 직접 지정한다.

---

## 3. 왜 고정 Joint Waypoint부터 시작하는가?

처음부터 IK, MoveIt 2, Vision 등을 모두 적용하면 하나의 동작이 실패했을 때 원인을 분리하기 어렵다.

예:

* IK 계산 문제
* 좌표계 문제
* TF 문제
* Trajectory 문제
* Joint Controller 문제
* Gripper 문제
* Gazebo Collision 문제
* 물리 파라미터 문제

따라서 먼저 **제어 파이프라인 전체가 정상적으로 동작하는 최소 기준선(Baseline)** 을 만든다.

```text
ROS2 Node
   ↓
Joint Command
   ↓
Gazebo
   ↓
Robot Arm
   ↓
Gripper
   ↓
Pick & Place
```

이 상태를 먼저 성공시킨다.

---

## 4. 단순 Joint Waypoint에 머무르지 않는다

고정 Joint Waypoint 방식만으로 프로젝트를 끝내지는 않는다.

고정된 Joint 값으로 동작하는 프로그램은 특정 환경에서 동작을 재현하는 데는 유용하지만, 로봇공학 학습 및 확장성에는 한계가 있다.

따라서 각 단계에서 **사람이 직접 결정하고 있는 부분을 하나씩 자동화**한다.

### 현재

```text
사람
 ↓
Joint 값 결정
 ↓
Robot
```

### 최종 방향

```text
작업 목표
 ↓
Target Pose
 ↓
IK
 ↓
Trajectory / Motion Planning
 ↓
Joint Controller
 ↓
Robot
```

더 나아가:

```text
Camera
 ↓
Object Detection
 ↓
Object Position
 ↓
TF2
 ↓
Target Pose
 ↓
IK
 ↓
Motion Planning
 ↓
Gripper
 ↓
Feedback
```

---

## 5. 단계별 발전 계획

### Phase 1 — Fixed Joint Waypoint

목표:

> 고정된 위치에서 `s` 입력만으로 Pick & Place 전체 동작을 성공시킨다.

구현:

* `s` 키 입력
* Pick & Place Controller
* State Machine
* Joint Position Command
* Gripper OPEN/CLOSE
* HOME 복귀

상태:

```text
INIT
 ↓
OPEN_GRIPPER
 ↓
HOME
 ↓
APPROACH_BALL
 ↓
LOWER_TO_BALL
 ↓
CLOSE_GRIPPER
 ↓
LIFT_BALL
 ↓
MOVE_ABOVE_BASKET
 ↓
LOWER_INTO_BASKET
 ↓
OPEN_GRIPPER
 ↓
RETREAT
 ↓
RETURN_HOME
 ↓
DONE
```

---

### Phase 2 — End-Effector Pose + IK

목표:

> Joint 값을 직접 지정하지 않고 목표 End-Effector Pose를 지정한다.

기존:

```text
approach_ball()
 → joint1, joint2, ...
```

변경:

```text
approach_ball()
 → target position + orientation
 → IK
 → joint angles
 → joint command
```

이를 통해 공의 위치가 조금 달라져도 대응할 수 있는 구조로 발전시킨다.

---

### Phase 3 — Trajectory / Motion Planning

목표:

> 시작 자세에서 목표 자세까지의 이동 경로를 로봇 관점에서 관리한다.

검토 기술:

* Joint trajectory
* Cartesian trajectory
* Velocity / acceleration
* Joint limits
* Collision checking
* MoveIt 2

단순히 최종 Joint 값만 계산하는 것이 아니라 **어떻게 그 자세까지 이동할 것인지**를 다룬다.

---

### Phase 4 — TF2 및 좌표계

목표:

> 공의 위치가 다른 좌표계에서 주어져도 로봇 기준 좌표로 변환할 수 있도록 한다.

예:

```text
camera frame
      ↓
     TF2
      ↓
base_link
      ↓
target pose
      ↓
IK
```

학습 내용:

* Coordinate Frame
* TF2
* Translation
* Rotation
* Quaternion
* Pose Transformation

---

### Phase 5 — Vision

목표:

> 공의 위치를 사람이 직접 입력하지 않고 센서로 획득한다.

예:

```text
Camera
 ↓
Object Detection
 ↓
Ball Position
 ↓
TF2
 ↓
Target Grasp Pose
 ↓
IK
```

향후 YOLO 및 Depth 정보를 활용할 수 있다.

---

### Phase 6 — Feedback 기반 Pick & Place

최종적으로는 단순한 명령 실행이 아니라 **현재 상태를 확인하면서 다음 행동을 결정하는 구조**를 목표로 한다.

예:

```text
CLOSE_GRIPPER
      ↓
공을 잡았는가?
   ↙       ↘
 YES        NO
 ↓           ↓
LIFT       RETRY
```

또는:

```text
MOVE_TO_BASKET
      ↓
공을 유지하고 있는가?
   ↙       ↘
 YES        NO
 ↓           ↓
CONTINUE   FAILURE
```

---

## 6. 그리퍼 초기화

작업 시작 시 그리퍼 상태를 가정하지 않고 **명시적으로 OPEN 상태로 초기화**한다.

```text
INIT
 ↓
OPEN_GRIPPER
 ↓
HOME
```

이는 이후 실제 로봇 및 반복 실행을 고려했을 때도 중요한 설계 원칙이다.

---

## 7. 공을 잡는 방법

초기에는 기존 RG2 그리퍼의 Joint Position Command를 이용해 OPEN/CLOSE를 제어한다.

공을 실제 물리 접촉으로 안정적으로 잡기 어려운 경우 Gazebo의 attach/detach 방식 등을 검토한다.

우선 목표는 **Pick & Place 전체 상태 머신과 제어 흐름을 검증하는 것**이며, 물리적으로 완벽한 grasp simulation은 이후 단계에서 개선한다.

---

## 8. MoveIt 2의 도입 시점

MoveIt 2를 처음부터 사용하지 않는다.

먼저 다음을 직접 경험한다.

```text
Joint
 ↓
FK / IK
 ↓
Pose
 ↓
Trajectory
```

그 이후 MoveIt 2를 도입하여:

```text
Target Pose
 ↓
IK
 ↓
Motion Planning
 ↓
Collision Avoidance
 ↓
Trajectory
 ↓
Controller
```

구조로 발전시킨다.

이렇게 함으로써 MoveIt 2를 단순히 사용하는 것에 그치지 않고 **MoveIt 2가 해결하는 문제가 무엇인지 이해하는 것을 목표로 한다.**

---

## 9. 기술 적용의 핵심 원칙

각 단계에서 다음 질문을 기준으로 기능을 발전시킨다.

> **"현재 사람이 직접 결정하고 있는 부분은 무엇이며, 이를 어떤 로봇공학 기술로 자동화할 수 있는가?"**

예:

| 현재 사람이 결정하는 것 | 이후 적용할 기술                    |
| ------------- | ---------------------------- |
| Joint 값       | IK                           |
| 이동 경로         | Trajectory / Motion Planning |
| 공 위치          | Vision                       |
| 좌표 변환         | TF2                          |
| 충돌 여부         | Collision Checking           |
| Grasp 성공 여부   | Feedback                     |
| 실패 시 다음 행동    | State Machine / Recovery     |

---

## 10. 최종 목표

최종적으로 다음과 같은 시스템을 목표로 한다.

```text
                    ┌──────────────┐
                    │   Keyboard   │
                    │      s       │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │ State Machine│
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │ Object Pose  │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │     TF2      │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │ Target Pose  │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │      IK      │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │Motion Planner│
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │   Controller │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │     Arm      │
                    └──────┬───────┘
                           ↓
                    ┌──────────────┐
                    │   Gripper    │
                    └──────┬───────┘
                           ↓
                       Feedback
                           │
                           └────────→ State Machine
```

최종적으로는 **공과 바구니의 위치가 로봇의 작업 범위 내에서 달라져도 Pick & Place를 수행할 수 있는 시스템**으로 발전시키는 것을 목표로 한다.

---

## 11. 이번 단계의 범위

현재 구현에서는 범위를 다음과 같이 제한한다.

### 구현한다

* `s` 키로 자동 시작
* Pick & Place State Machine
* Gripper OPEN 초기화
* Fixed Joint Waypoint
* 공 집기
* 공 이동
* 바구니에 놓기
* HOME 복귀

### 아직 구현하지 않는다

* IK
* MoveIt 2
* Vision
* YOLO
* Dynamic Object Detection
* 복잡한 Collision Avoidance
* 자동 Grasp Planning

이 기능들은 **Phase 1이 안정적으로 동작한 이후 순차적으로 추가한다.**

---

## 12. 결정 이유 요약

**결정: `Fixed Joint Waypoint → IK → Planning → TF2 → Vision → Feedback` 순으로 단계적 발전**

이유:

1. 먼저 전체 ROS2-Gazebo-Controller-Gripper 파이프라인을 검증할 수 있다.
2. 각 문제를 독립적으로 디버깅할 수 있다.
3. IK와 Motion Planning의 필요성을 실제 프로젝트 안에서 체감할 수 있다.
4. 단순 동작 재현 프로젝트에서 실제 로봇 시스템으로 점진적으로 발전시킬 수 있다.
5. 각 단계가 로봇공학 학습 주제와 직접 연결된다.
6. 최종적으로 위치가 변하는 물체에 대응하는 Pick & Place 시스템으로 확장할 수 있다.

**따라서 첫 구현은 단순하게 시작하되, 프로젝트의 설계 방향은 처음부터 "수동으로 결정한 값을 하나씩 로봇공학 알고리즘으로 대체한다"는 방향으로 유지한다.**
