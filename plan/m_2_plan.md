# MoveIt 적용 2단계 상세 설계

## 1. 목적

이 문서는 [`moveit_apply.md`](moveit_apply.md)의 단계 2인 **MoveIt 설정 패키지와 RViz 계획 검증**을 구현 가능한 단위로 구체화한다. 단계 1에서 확정한 환경과 모델 기준을 재사용하고, 단계 1에서 미완료로 남긴 SRDF·자기충돌행렬 검토와 바구니 내부 경계 조사를 선행 작업으로 완료한다.

단계 2의 결과는 Gazebo 실행과 독립적으로 시작할 수 있는 MoveIt `move_group` 및 RViz MotionPlanning 구성이다. 이 단계에서는 경로를 계획·검증하지만 Gazebo 로봇에 궤적을 전송하지 않는다. ros2_control 실행 연동, 접촉 감지, 물체 부착/분리 서비스, 전체 작업 노드는 각각 후속 단계의 범위다.

## 2. 목표와 완료 기준

### 목표

- URDF/Xacro 모델을 MoveIt 로봇 모델로 사용하고 SRDF 의미 모델을 확정한다.
- `ur5` 팔 그룹의 시작 자세, 공 접근 자세, 바구니 위 자세에 대해 충돌을 고려한 유효 계획을 RViz에서 생성한다.
- Gazebo SDF와 일치하는 공·바구니를 Planning Scene에 표현하고, 접근·이동 경로의 충돌 여부를 검토한다.
- 재현 가능한 패키지 설정과 독립 실행 절차를 마련한다.

### 완료 기준

1. MoveIt 설정 패키지가 빌드되고 설치되며 `move_group` 및 RViz MotionPlanning이 실행된다.
2. 로봇 모델이 `world` planning frame, `base_link` arm chain base, `rg2_hand` end-effector tip 기준으로 표시된다.
3. UR5 그룹의 `home` 상태에서 지정된 시험 목표까지 계획이 성공하거나, 불가능한 목표의 기하학적/충돌 원인이 기록된다.
4. self-collision matrix의 각 제외 쌍에 근거가 있고, 안전상 허용할 수 없는 링크 쌍은 제외되지 않는다.
5. Planning Scene의 공 및 바구니 충돌 형상·pose가 월드 SDF 기준과 일치하고, 공은 접근 시 충돌 대상으로 작동한다.
6. 단계 2 실행에 Gazebo 또는 trajectory controller가 필요하지 않으며, 실제 궤적 실행이 비활성 상태임이 명확하다.

## 3. 입력 기준 및 미결 항목

### 3.1 확정 기준

`m_1_plan.md`와 `moveit_apply.md`의 조사 결과를 기준으로 한다.

| 항목 | 기준 |
|---|---|
| OS / ROS / Gazebo | Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic |
| MoveIt | Jazzy 바이너리 2.12.4 계열 |
| URDF 소스 | `my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro` |
| planning frame | `world` |
| arm group | `ur5`, 6개 UR5 revolute joint |
| gripper group | `gripper`, `rg2_finger_joint1`, `rg2_finger_joint2` |
| end-effector | `rg2_hand` 링크, 부모 group `ur5`, 연결 링크 `wrist_3_link` |
| 보조 프레임 | `tool0`를 별도 참조 프레임으로 유지; `rg2_hand`와 혼용하지 않음 |
| 고정 베이스 | URDF의 `world`–`base_link` fixed joint 사용; SRDF virtual joint를 추가하지 않음 |
| 명목 home | 팔 6축 및 finger 2축 모두 0 rad; 실제 상태와 비교해 허용 오차 기록 |
| 공 | SDF pose `(0.55, -0.25, 0.0325)` m, 구 반지름 `0.0325` m |
| 바구니 | SDF 모델 pose `(0.75, 0.25, 0.000022)` m; mesh AABB만으로 유효 내부를 단정하지 않음 |

URDF의 팔 joint 한계는 각 축 `[-6.28319, 6.28319]` rad, 손가락 한계는 각각 `[0, 1.18]`, `[-0.001, 1.18]` rad다. 계획에 적용되는 유효 joint limit은 URDF 파싱 결과를 기준으로 하되, 별도 MoveIt joint limits 설정을 둘 경우 그 제한이 더 느슨해지지 않게 한다.

### 3.2 단계 1에서 이월된 필수 선행 조사

아래 조사 결과가 완료되기 전에는 설정 생성물을 최종 확정하지 않는다.

1. **SRDF 그룹·자세 확인**: Setup Assistant로 모델을 로드하고 `ur5`, `gripper`, `ur5_rg2_eef`, `home`을 생성한다. 그룹 관절 목록, 그룹 계층, EEF 부모/연결 링크, 자세의 모든 관절값을 표로 저장한다. 생성 SRDF를 수동 검토해 고정 프레임을 virtual joint로 중복 선언하지 않았는지 확인한다.
2. **자기충돌행렬 검토**: 대표적인 무작위 자세 샘플링으로 행렬을 생성한다. 생성 로그와 sampling 조건을 기록하고, 제외된 link pair 전체를 검토한다. 인접 링크 또는 기구상 항상 겹치는 pair만 허용하며, 손가락-공 같은 작업 중 접촉 대상은 self-collision 행렬에서 처리하지 않고 별도 ACM/attached-object 시점에서 관리한다. 임의로 모든 쌍을 허용하지 않는다.
3. **바구니 내부 경계 조사**: `my_arm_with_ball_description/models/...`의 바구니 원본 mesh 위치를 확인한다(계획서 기준 `my_arm_with_ball_gazebo` world가 참조하는 모델). SDF의 모델/link pose와 mesh 좌표계를 합성해 입구 평면, 내부 유효 폭/깊이, 벽 두께, 바닥 높이를 추출한다. 단순 AABB는 초기 충돌 proxy 산정에만 쓰고, 바구니 내부 목표점/공 containment 판정으로 간주하지 않는다.
4. **목표 pose 확정**: `rg2_hand` 기준으로 공 접근/파지 전 목표, 수직 lift 목표, 바구니 중심 위 놓기 목표를 정한다. SDF 월드 좌표와 `world` frame을 사용한다. 방향은 접근 방향과 손가락 폐쇄 축을 고려해 정하고, 공·바구니 형상 및 그리퍼 충돌을 RViz에서 확인한다. 자세를 근거 없이 임의 지정하지 말고 작업 가능 후보와 변환을 함께 기록한다.

단계 1 조사 항목 가운데 바구니 내부 경계와 계획 포즈는 조사·튜닝 성격상 이 문서에서 수행하며, 놓기 판정의 안정 속도/시간은 단계 6으로 이월한다.

## 4. 패키지 및 파일 설계

새 패키지는 기존 역할을 확장하기보다 MoveIt 설정 전용 패키지로 둔다.

```text
my_arm_with_ball_moveit_config/
  CMakeLists.txt
  package.xml
  config/
    ur5_rg2.srdf
    kinematics.yaml
    joint_limits.yaml
    ompl_planning.yaml
    moveit_controllers.yaml       # 후속 실행 연동 때 활성화/완성
    moveit.rviz
    planning_scene.yaml           # 정적 물체 pose/geometry가 설정 파일로 적합할 때
  launch/
    move_group.launch.py
    moveit_rviz.launch.py
    demo.launch.py                # move_group + RViz, fake state/controller
```

설치 가능한 `moveit_configs_utils` 패턴 및 현재 Jazzy 템플릿을 우선 사용한다. 별도 URDF 복사본은 두지 않고 원본 Xacro를 패키지 share 경로에서 읽어 launch 시 확장한다. 이로써 Gazebo/Xacro 모델과 MoveIt robot description의 이중 수정 위험을 줄인다. 실제 Jazzy 설치 템플릿이 이 구조를 지원하지 않는 항목은 공식 패키지 API와 생성 패키지 예제에 맞춰 조정하고, 결정 및 파일 목록을 결과에 남긴다.

기존 bringup에는 단계 2에서 필수 변경을 최소화한다. 실행 책임은 `my_arm_with_ball_moveit_config`가 가진다. 이후 통합 시에만 `my_arm_with_ball_bringup`에서 공통 시뮬레이션과 MoveIt을 함께 시작하는 launch를 추가하는 방안을 검토한다.

## 5. SRDF 의미 모델

### 5.1 그룹

- `ur5`: `base_link`에서 `rg2_hand`까지의 chain으로 정의하고 6개 팔 joint가 정확히 한 번 포함되는지 검증한다. Xacro의 중간 fixed joint와 gripper 연결 fixed joint를 통한 chain이 유효한지 MoveIt 모델에서 확인한다.
- `gripper`: `rg2_finger_joint1`, `rg2_finger_joint2` 두 관절을 명시적으로 포함한다. 두 joint가 독립 모델이므로 현재 단계에서 mimic 관계를 만들지 않는다.
- `ur5_rg2_eef`: `ur5` parent group, `rg2_hand` parent link, EEF link `rg2_hand`로 등록한다. SRDF parser/Setup Assistant에서 허용하는 최소 일관 정의를 사용한다. 필요 시 `gripper`를 포함하는 group 구성은 실제 MoveIt UI에서 독립 그룹 선택/EEF 해석을 확인한 후 확정한다.

SRDF 그룹 정의가 실제 계획 관절 집합에 미치는 결과를 `move_group`의 robot model 로드 로그와 RViz group 목록으로 대조한다. 모호한 겹침 또는 URDF에 없는 이름이 있으면 설정을 완료 처리하지 않는다.

### 5.2 자세

- `home`은 `ur5`와 `gripper`에 정의한다. 관절값은 명목 0 rad로 두고, 8개 관절 이름과 값이 완전한지 확인한다.
- MoveIt named state는 group별로 정의하므로 팔과 그리퍼의 `home` 적용 결과를 각각 시험한다. 전체 로봇 시작 상태가 필요한 곳에서는 양 group 상태를 함께 적용한다.
- 공 접근/파지, lift, 바구니 위 자세는 먼저 named state로 과도하게 고정하지 않는다. 실제 목표 pose를 planning frame에서 지정해 도달성 검증하고, 반복 가능한 안전 자세로 확정된 경우에만 추가한다.
- 시작 상태 오차 허용치는 `/joint_states` 수치 오차에 기반해 정한다. 0으로 간주한 finger 미세 오차와 플래너 상태 유효성에 영향을 주는 tolerance를 구분해 기록한다.

### 5.3 자기충돌행렬

Setup Assistant의 self-collision matrix 생성은 후보 행렬을 만드는 절차다. 자동 생성 결과를 그대로 승인하지 않는다.

검토 기록에는 (a) link pair, (b) 자동 비허용 사유/인접성, (c) 실제 모델에서 충돌 가능성, (d) 승인/복원 판정, (e) 근거를 남긴다. 팔 접힘 자세, 뻗은 자세, 그리퍼 열림/닫힘 자세를 포함해 RViz에서 유효 상태와 의도하지 않은 관통이 없는지 확인한다. 정상적으로 서로 닿을 수 있지만 기능상 접촉이 필요한 finger-공 조합은 scene ACM 또는 attached object 허용 충돌 규칙으로 다룰 것이며, 전체 self-collision matrix에 영구 예외를 넣지 않는다.

## 6. MoveIt 설정

### 6.1 Kinematics

`ur5` group에 Jazzy에서 제공되는 KDL kinematics plugin을 기본으로 설정한다. 탐색 해상도와 timeout은 기본값에서 시작하고, 접근 목표가 풀리지 않을 때만 로그와 반복 결과를 근거로 조정한다. IK timeout/attempt 수를 변경하면 값과 이유를 결과 문서에 기록한다. `gripper`는 팔 Cartesian IK planning group으로 사용하지 않는다.

### 6.2 Joint limits

URDF 한계를 원천값으로 유지한다. `joint_limits.yaml`은 속도/가속도 제한처럼 MoveIt 전용 보완이 필요할 때만 명시하며, URDF보다 넓은 위치 한계를 설정하지 않는다. 실제 시뮬레이터 한계와 모델 선언이 불일치하면 단계 2에서 임의로 모델을 고치지 않고 문제를 기록해 범위 결정으로 올린다.

### 6.3 Planning pipeline

- OMPL을 우선 planning pipeline으로 구성한다.
- planner는 설치된 Jazzy MoveIt 기본 OMPL 설정과 호환되는 기본 sampling planner로 시작한다(예: `RRTConnect`). 별도 planner를 추가하기 전에 목표별 반복 계획 성공 여부를 확인한다.
- planning time, attempts, velocity/acceleration scaling은 명시적으로 재현 가능한 설정으로 관리한다. 초기에는 보수적 스케일링을 적용하고, 단계 2에서는 실제 로봇 실행을 하지 않는다.
- 경로 검증은 goal pose 도달, trajectory 유효성, self/world collision 부재를 각각 구분해 기록한다. 단순히 “planner success” 한 항목으로 대체하지 않는다.

수치 기본값은 구현 시 설치된 Jazzy 기본 설정과 실제 계획 시간을 확인한 뒤 yaml에 고정한다. 값이 아직 검증되지 않은 상태를 계획서에서 확정 수치처럼 가장하지 않는다.

## 7. Planning Scene 설계

### 7.1 물체

- `apple`(SDF 모델 이름은 확인 후 실제 이름에 맞춤): 월드 pose를 `(0.55, -0.25, 0.0325)` m에 놓는다. 우선 반지름 0.0325 m 구 collision primitive를 사용한다. mesh가 구와 다르거나 SDF collision pose가 다르면 원본 SDF를 우선하고 모델 편차를 기록한다.
- `basket`: 모델 기준점 `(0.75, 0.25, 0.000022)` m와 방향을 반영한다. 충돌 장면은 내부 공간을 막는 단일 solid box로 만들지 않는다. mesh collision 또는 바닥/벽을 나타내는 여러 primitive로 내부가 빈 형상을 구성한다. 근사 형상은 바구니 내부 목표 영역을 보존하고 실제 벽과 공의 충돌을 막아야 한다.
- 모든 object ID는 상수화하고 고정 pose는 SDF 기준값과 대조한다. `world`가 planning frame이므로 추가 변환이 없다.

충돌 mesh 사용과 primitive 구성 중 어떤 표현을 쓸지는 실제 basket mesh의 파싱/계산 비용, 내부 형상 보존 여부를 비교해 결정한다. 설정은 하나의 최종 표현만 활성화하며 중복 geometry로 충돌 공간을 이중화하지 않는다.

### 7.2 초기/접촉/부착 상태

단계 2에서는 다음 세 장면을 계획 검증용으로 구분한다.

1. **초기 scene**: 공과 바구니가 고정 world collision object로 존재. 공 접근 전 경로 계획에 사용.
2. **접근 목표 검증 scene**: 공은 여전히 world object로 남긴다. 손가락/hand가 공과 겹치는 목표는 일반 충돌 검사에서 실패해야 한다. 파지 자세의 충돌 허용은 단계 4의 접촉 및 부착 동작 설계와 일치시킬 규칙만 별도 기술하고, 자동으로 전체 충돌을 무시하지 않는다.
3. **attached object 가상 검증 scene**: 단계 2의 정적 계획 확인이 필요할 때만 공을 `rg2_hand`에 붙인 상태를 RViz에서 수동/테스트 scene으로 만들어 lift 및 바구니 이동 경로를 확인한다. 실제 Gazebo attach/detach 구현은 하지 않는다. 부착 시 touch links는 실제로 공을 잡는 `rg2_hand`, 손가락 link로 최소화하며 주변 팔 링크를 넓게 허용하지 않는다.

초기 실행에서 정적 물체를 자동 등록하는 방법은 `PlanningScene` diff를 publish하는 launch 보조 노드 또는 초기 scene 설정 중 한 가지를 선택한다. 후속 단계에서 scene 갱신 인터페이스를 제공할 수 있도록 ROS API 의존성과 object ID를 기록한다. planning scene이 매 실행마다 안정적으로 초기화되고 RViz/MoveGroup 양쪽에서 같은 상태를 보는지 확인한다.

## 8. RViz와 독립 실행

- `moveit.rviz`에는 RobotModel, MotionPlanning, TF 표시를 포함한다. 계획 scene의 공·바구니 충돌 geometry를 시각적으로 구분한다.
- `demo.launch.py`는 Xacro 기반 `robot_description`, SRDF 기반 `robot_description_semantic`, kinematics/limits/planning pipeline, `move_group`, RViz를 시작한다. Gazebo를 시작하지 않고 fake joint state/controller를 사용해 계획 전용으로 동작한다.
- 실제 Gazebo joint state와 컨트롤러를 연결하는 launch는 이 단계 범위 밖이다. launch 이름/인자에서 demo와 simulation 계획 검증을 명확히 구분한다.
- `use_sim_time` 및 clock 입력은 독립 계획 실행에서 필수 조건으로 만들지 않는다. 후속 통합 launch에서 Gazebo clock을 전달하는 방식은 단계 3에서 결정한다.
- launch 종료 시 전체 프로세스가 정상 정리되는지 확인하고, 모델 로드·TF·planning scene 초기화 오류를 구별해 로그에서 파악할 수 있어야 한다.

## 9. 시험 계획 및 합격 기록

실제 구현 시 아래 시험을 순서대로 수행한다. 각 결과에는 실행 명령, 목표 pose/joint 값, planning frame, 플래너, 성공 여부, planning time, 실패 원인을 기록한다.

| ID | 시나리오 | 합격 기준 |
|---|---|---|
| P2-01 | MoveIt 설정 패키지 빌드 및 설치 | 패키지 오류 없이 완료, config/launch가 share에 설치됨 |
| P2-02 | 독립 `demo.launch.py` 시작 | `move_group`, RViz 기동; robot model/SRDF/TF 오류 없음 |
| P2-03 | 그룹·자세 확인 | RViz에 `ur5`, `gripper`; `home` 값이 설정과 일치 |
| P2-04 | 시작 상태 유효성 | `home`에서 self-collision 및 world collision 없음 |
| P2-05 | 공 접근 계획 | 충돌을 피하면서 접근 목표에 유효한 궤적 생성; 파지 목표 충돌 시 원인이 설명됨 |
| P2-06 | lift/바구니 위 계획 | 초기 scene 또는 명시된 attached scene에서 경로가 충돌 없이 계획됨 |
| P2-07 | 바구니 내부 표현 | 내부에 빈 공간이 보존되고 공 중심 목표 영역을 과도하게 막지 않음 |
| P2-08 | 음성 대조 | 바구니 벽/공에 의도적으로 겹치는 목표는 collision-invalid 또는 계획 실패로 판정 |
| P2-09 | scene 동기화 | RViz, MoveGroup의 object ID, pose, attached 상태가 일치 |
| P2-10 | 실행 경계 | 계획만 수행하고 Gazebo controller/action으로 trajectory가 전송되지 않음 |

실패한 시험도 제거하지 않고 결과와 원인을 남긴다. 도달 불가가 확인된 포즈는 성공 조건을 완화해 감추지 않고, frame/EEF transform 오류, 목표 기하학, 관절 한계, 충돌 중 원인을 분류한다.

## 10. 구현 순서

1. 선행 조사 완료: SRDF 후보 정의, self-collision pair 검토, 바구니 mesh 내부 경계/pose 확인, `rg2_hand` 목표 pose 초안 작성.
2. 새 설정 패키지 생성: 의존성, 설치 대상, Xacro source 전달, launch entry point 구성.
3. SRDF 생성 및 정적 검토: group, EEF, `home`, self-collision matrix를 확정하고 diff를 점검.
4. MoveIt 설정 작성: kinematics, joint limits, OMPL, RViz 설정과 독립 계획 launch를 연결.
5. Planning Scene 물체 구현: 공과 바구니 collision geometry, 안정적인 초기 등록 방법 구현.
6. RViz 계획 검증: home 유효성, 공 접근, lift, 바구니 위, 충돌 음성 대조 시나리오 수행.
7. 설계/구현 결과 갱신: 실제 파일 목록, 확정 파라미터, 알려진 제한 및 다음 단계 인계 항목을 기록.

## 11. 단계 2의 범위 밖 및 후속 단계 인계

- Gazebo trajectory controller를 MoveIt controller manager에 연결하고 계획 궤적을 실행하는 것: 단계 3.
- 접촉 센서, gripper closed 판정, Gazebo 공 attach/detach, Planning Scene의 실시간 동기화: 단계 4.
- 전체 pick-and-place 상태 기계 및 자동 작업 요청 API: 단계 5.
- 낙하 안정도와 바구니 containment 성공 판정: 단계 6.

단계 2의 attached-object scene은 계획 검증용이다. 이 결과만으로 물리적으로 파지 성공이나 Gazebo 부착 성공을 주장하지 않는다.

## 12. 결과 기록

구현 후 아래 항목을 이 절에 갱신한다.

- 생성 파일과 패키지 의존성
- SRDF 그룹/자세의 실제 joint 목록 및 self-collision matrix 승인/복원 내역
- 바구니 내부 기준점·입구 크기·충돌 표현 및 출처
- 확정된 시험 목표 pose와 `rg2_hand` 변환
- kinematics/planner/limits 설정과 근거
- P2-01~P2-10 결과, 실행 명령 및 검증 로그 요약
- 발견한 제한, 미완료 항목, 단계 3으로 전달할 controller/scene 인터페이스

### 구현 결과 (2026-10-12)

#### 생성물

- 새 패키지 `my_arm_with_ball_moveit_config`를 추가했다. 로봇 URDF는 복제하지 않고 `my_arm_with_ball_description`의 Xacro를 MoveIt 설정 생성기가 읽도록 했다.
- SRDF에는 `ur5` chain (`base_link` → `rg2_hand`), 독립 관절 2개로 구성한 `gripper`, `ur5_rg2_eef`, 각 group의 `home` 자세를 정의했다. virtual joint는 두지 않았다.
- `kinematics.yaml`은 KDL, `ompl_planning.yaml`은 RRTConnect를 사용한다. `longest_valid_segment_fraction`은 basket 근처의 얇은 충돌면을 놓치지 않도록 `0.0005`로 설정했다. `joint_limits.yaml`은 URDF position/velocity limit을 재정의하지 않고 demo acceleration/scaling만 보완한다.
- headless/GUI 공용 `demo.launch.py`, `move_group.launch.py`, `moveit_rviz.launch.py` 및 RViz MotionPlanning 설정을 추가했다. `allow_trajectory_execution=false`이고 `MoveGroupExecuteTrajectoryAction` capability를 비활성화한다.
- 시작 시 `ApplyPlanningScene` service로 공을 `apple` 구(`r=0.0325 m`), 바구니를 mesh collision object로 `world`에 등록한다. 바구니 mesh는 GoogleResearch 원본 OBJ의 삼각형 표면을 좌표 변경/감소 없이 binary STL로 변환해 패키지 안에 넣었으며 CC BY 4.0 출처를 `meshes/ATTRIBUTION.md`에 기록했다. 위치는 원본 world SDF와 같은 `(0.75, 0.25, 0.000022)` m다.
- `scripts/validate_planning.py`는 설치되는 계획 전용 검증 스크립트다. 실제 로봇 또는 Gazebo controller로 trajectory를 전송하지 않는다.

#### SRDF 및 충돌행렬 판정

- MoveIt 2.12.4의 Setup Assistant `computeDefaultCollisions` 알고리즘에 Xacro 확장 URDF, 위 SRDF, description package mesh를 공급해 10,000회 무작위 자세 샘플을 생성했다. 전체 링크 쌍 66개 중 `Adjacent` 9쌍, `Never` 후보 10쌍, `Sometimes` 47쌍으로 분류됐고 `Always`/`Default`는 0쌍이었다.
- `Adjacent` 9쌍은 실제 URDF joint tree에서 직접 연결된 링크이므로 충돌검사 제외를 승인했다: `base_link–shoulder_link`, `shoulder_link–upper_arm_link`, `upper_arm_link–forearm_link`, `forearm_link–wrist_1_link`, `wrist_1_link–wrist_2_link`, `wrist_2_link–wrist_3_link`, `wrist_3_link–rg2_hand`, `rg2_hand–rg2_leftfinger`, `rg2_hand–rg2_rightfinger`.
- `Never` 후보 10쌍은 SRDF에 추가하지 않았다. 이는 유한한 표본에서 충돌이 관찰되지 않았다는 결과이므로, finger 간 접촉이나 드물게 가능한 비인접 충돌을 놓치지 않도록 계속 검사한다. 결과와 pair 목록은 실행 시 `/tmp/stage2_collision_matrix.txt`에 남겼다.

#### 검증 결과

명령은 ROS Jazzy 환경에서 `colcon build --packages-select my_arm_with_ball_moveit_config --symlink-install`로 새로 빌드했다. 빌드는 성공했다. `demo.launch.py`를 Gazebo 없이 실행해 `move_group`, Xacro/SRDF model, KDL, OMPL, scene loader 및 RViz MotionPlanning 패널이 시작되는 것을 확인했다. RViz 화면에서 UR5, 공, 속이 빈 바구니 mesh가 보이고 interactive planning marker가 연결됐다.

재현 명령:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select my_arm_with_ball_moveit_config --symlink-install
source install/setup.bash
ros2 launch my_arm_with_ball_moveit_config demo.launch.py
```

다른 터미널에서 live planning service 검증을 반복하려면 `ros2 run my_arm_with_ball_moveit_config validate_planning.py`를 실행한다. 스크립트는 계획 응답과 보간 상태를 확인하고 실행 trajectory를 controller로 전송하지 않는다.

| ID | 결과 | 증거 |
|---|---|---|
| P2-01 | 통과: 새 패키지 build/install | `colcon build --packages-select my_arm_with_ball_moveit_config --symlink-install` |
| P2-02 | 통과: 독립 demo, move_group, RViz MotionPlanning 기동 | `/tmp/stage2_demo.log`; RViz가 model을 로드하고 interactive marker 연결 |
| P2-03 | 통과: 그룹·자세·EEF model 로드 | SRDF 및 RViz group `ur5`; live joint-state 모델 로드 |
| P2-04 | 통과: 8개 joint가 0 rad인 home의 state validity 결과 valid | `/tmp/stage2_planning_validation.log` |
| P2-05 | 통과: 공 접근 목표 계획 | world, `rg2_hand` `(0.55, -0.25, 0.3325)` m; 최신 RViz demo 검증에서 계획 0.362 s, 보간 상태 394개 검사 |
| P2-06 | 통과: lift 후 basket 위 목표 계획 | lift `(0.55,-0.25,0.45)` m 및 basket above `(0.75,0.25,0.4)` m; 최신 검증에서 각 0.291/0.356 s, 보간 상태 566/226개 검사 |
| P2-07 | 통과: basket 내부를 보존하는 원본 삼각 mesh 사용 | full collision surface mesh가 RViz에서 빈 내부와 함께 표시됨 |
| P2-08 | 통과: 음성 대조 | 손을 공 중심에 두거나 basket 벽과 겹치면 상태 invalid, 각각 apple/basket contact 반환 |
| P2-09 | 통과: Planning Scene 동기화 | `get_planning_scene`에서 `apple` 구와 `basket` mesh의 ID·world pose 확인 |
| P2-10 | 통과: 계획 전용 경계 | `/move_group allow_trajectory_execution=False`; `/execute_trajectory`에는 action server가 없고 Gazebo/controller도 실행하지 않음 |

최신 세 목표의 최종 FK 위치 오차는 최대 약 0.77 mm, 방향 오차는 최대 약 0.0014 rad였다. 계획기는 RViz UI 버튼 조작 대신 같은 `move_group` 계획 서비스를 사용하는 설치 스크립트로 목표 계획을 반복 가능하게 검증했고, 실행 중 RViz에서 같은 `DisplayMotionPath` 경로와 scene을 시각적으로 확인했다. 실행 파일 `/tmp/stage2_rviz_validation.log`에 최신 결과가 있다.

#### 제한 및 후속 단계 인계

- 직접 연결되지 않은 `Never` pair 10개는 안전 여유를 위해 제외하지 않았다. 성능상 필요가 생길 때 개별 검토 대상으로 둔다.
- attached-object 상태의 이동 경로는 이 단계에서 검증하지 않았다. 세 경로 계획 시 공은 world object로 남아 있었고, 실제 부착/분리는 단계 4에서 Gazebo와 Planning Scene 동기화로 구현한다.
- 바구니의 정확한 containment 입구 치수 및 공 안정 판정은 정하지 않았다. collision mesh 전체 표면을 충돌 모델로 사용했고, 놓기 성공 기준 수치는 단계 6에 남긴다.
- MoveIt은 기본 controller-manager plugin을 로드하지만 controller 목록은 비어 있다. 실행 action capability는 제거되어 있으며 Gazebo 실행 연동은 단계 3에서 한다.
- 정상 종료 시 launch가 MoveGroup 및 RViz 프로세스의 종료를 기다리다 SIGTERM/SIGKILL로 승격되는 현상을 이 환경에서 관찰했다. 종료 후 관련 프로세스는 남지 않았다. 계획 실행 기능과는 별개지만 후속 통합에서 launch 종료 동작을 다시 확인한다.
- 계측 log는 현재 개발 머신 `/tmp/stage2_demo.log`, `/tmp/stage2_planning_validation.log`, `/tmp/stage2_collision_matrix.txt`에 있다. 장기 보관이 필요한 숫자와 결론은 이 문서 표에 옮겼다.
