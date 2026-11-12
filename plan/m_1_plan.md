# MoveIt 적용 1단계 상세 설계

## 1. 목적과 범위

이 문서는 [`moveit_apply.md`](moveit_apply.md)의 1단계인 **버전 호환성과 로봇 모델 기준 확정**을 위한 조사·판정 절차를 정의한다. 1단계에서는 패키지나 로봇 모델을 수정하지 않고, 이후 MoveIt 설정에 사용할 환경 조합, 로봇 그룹·프레임·관절 기준, 월드 목표 좌표, 모델 보완 항목을 증거와 함께 확정한다.

대상은 현재 개발 환경의 ROS 2 Jazzy, Gazebo Harmonic, `ros_gz`, MoveIt 2를 사용하는 시뮬레이션이다. 비전, 동적 목표물, 실물 로봇, 컨트롤러 실행 연동은 이번 단계 범위가 아니다.

## 2. 조사 시점 기준선

조사 기준은 현재 저장소와 이 작업 환경이며, 설치 패키지 버전은 시스템 업데이트에 따라 바뀔 수 있다.

| 항목 | 확인된 기준 | 증거 |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS (Noble) | `/etc/os-release` |
| ROS 2 | Jazzy (`ROS_DISTRO=jazzy`, `/opt/ros/jazzy`) | 셸 환경 및 설치 prefix |
| Gazebo | Harmonic, `gz sim` 8.15.0 (`GZ_VERSION=harmonic`) | 셸 환경 및 `gz sim --versions` |
| ros_gz | `ros-jazzy-ros-gz` 1.0.24-1 계열; bridge/sim 패키지 설치 | `dpkg-query` 패키지 목록 |
| gz_ros2_control | 1.2.20-1 계열 | `ros-jazzy-gz-ros2-control` 패키지 |
| MoveIt 2 | Jazzy 바이너리 설치; core/config utils/kinematics/OMPL/planning/RViz/Setup Assistant 2.12.4-1 계열 | `ros-jazzy-moveit-*` 설치 목록 및 package prefix |
| 모델 설명 | 별도 Xacro가 MoveIt용 URDF를 제공하고, Gazebo 물리 로봇은 별도 SDF를 사용 | `my_arm_with_ball_description/urdf/ur5_rg2.urdf.xacro`, `models/ur5_rg2/model.sdf` |
| 로봇 기준 배치 | 월드 SDF에서 로봇 pose는 원점, Xacro의 `world`–`base_link` 고정 변환도 항등 | `apple_pick_place.sdf`, Xacro 21–25행 |

패키지 숫자는 확인일의 로컬 설치값으로 기록한다. 외부 호환성의 기준은 공식 지원 조합표와 ROS 패키지 설치 가이드다. 공식 Gazebo 조합표는 ROS 2 Jazzy + Gazebo Harmonic을 권장 조합으로 표시하며, MoveIt 문서는 Jazzy를 지원 ROS 배포판으로 안내한다. 따라서 이 저장소에서는 기존 Jazzy/Harmonic 환경을 유지하고 각 컴포넌트의 공식 Jazzy 패키지를 사용한다. [Gazebo ROS 설치·호환표](https://gazebosim.org/docs/latest/ros_installation/), [MoveIt 2 시작 안내](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

## 3. 확정할 로봇 모델 기준

### 3.1 Planning group 초안

1단계에서는 아래 그룹을 후속 SRDF 설계의 기준으로 확정한다. 실제 SRDF 파일 생성은 2단계에서 한다.

| 그룹 | 구성 | 용도 |
|---|---|---|
| `ur5` | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` | MoveIt의 주 작업공간 계획 그룹 |
| `gripper` | `rg2_finger_joint1`, `rg2_finger_joint2` | 손가락 상태 표시 및 이후 그리퍼 제어/상태 모델링 |

두 finger joint는 서로 반대 축 방향으로 정의되어 있으며 별개의 관절이다. 이 단계에서는 mimic 관계를 가정하거나 Xacro를 바꾸지 않는다. 양쪽 관절 모두 독립 관절로 취급하는 모델이 현재 Xacro·SDF 정의와 맞는지 비교해 확정한다.

기본 자세 이름과 수치는 모델·시뮬레이터의 실제 시작 관절 상태를 관찰해 정한다. 키보드 노드의 0 rad 재설정 목표를 Gazebo의 최초 자세와 같다고 가정하지 않는다. 기존 ros2_control 계획도 이 차이를 명시한다(`plan/ros2_control적용계획.md` 100–103행).

### 3.2 프레임 기준

- Xacro의 root link는 `world`이고, `base_link`는 항등 변환의 URDF fixed joint로 연결된다. MoveIt Setup Assistant가 이 모델을 읽고 SRDF virtual joint가 없는 상태를 fixed root로 처리하는 것을 확인했다. 따라서 SRDF에 `world`–`base_link` virtual joint를 추가하지 않는다.
- MoveIt 모델의 planning frame은 root link인 `world`로 정한다. UR5 planning group의 chain base는 `base_link`이며 여섯 개 팔 joint를 포함한다. 월드 SDF의 작업 좌표를 planning frame 변환 없이 쓸 수 있다.
- RG2의 작업 기준 링크/팔 group tip은 `rg2_hand`로 정한다. `tool0`는 보조 참조 프레임으로 유지한다. 0 관절 상태에서 `world → tool0`는 `(0, 0.384, 1.001)` m, RPY `(-1.571, 0, 0)` rad이고, `world → rg2_hand`는 `(0, 0.191, 1.001)` m, RPY `(1.571, 0, 1.571)` rad다. 두 프레임은 같은 위치가 아니므로 `tool0`를 그리퍼 기준점으로 간주하지 않는다.
- Gazebo의 초기 `/joint_states`와 `robot_state_publisher` TF에서 `world → base_link`가 항등 변환임을 확인했다. 최초 팔 상태는 여섯 축 모두 0 rad(부동소수점 오차 범위)이며, RG2 두 joint는 약 `+1.73e-5`, `-1.73e-5` rad로 사실상 0 rad다. 이름 있는 SRDF `home` 자세는 이 명목값에서 정의한다.

### 3.3 관절 한계와 충돌 형상

Xacro의 팔 6축은 각각 `[-6.28319, 6.28319] rad`와 관절별 속도·effort 값을 선언한다(109–129행). RG2 finger 두 축은 각기 `[0, 1.18] rad`, `[-0.001, 1.18] rad` 한계를 갖는다(134–144행). 이 값들이 SDF와 일치하는 기본값인지, MoveIt 계획에 적용할 실제 유효 한계인지 확인한다.

체크 항목:

1. Xacro를 URDF로 확장해 모든 link, joint, limit가 유효한지 확인한다.
2. `test_model_consistency.py`의 기존 비교 기준처럼 SDF/Xacro 링크, mesh, 관절 부모·자식, 원점, 축, 한계가 일치하는지 확인한다.
3. 충돌 mesh가 MoveIt의 URDF parser에서 로드되고 단위·위치·방향이 올바른지 검사한다. 로봇 mesh가 복잡하거나 계획 속도에 영향을 주면 후속 단계에서 단순 collision geometry 도입 여부를 결정한다. 1단계에서 원본 mesh를 임의 교체하지 않는다.
4. 연속 회전 조인트로 취급해야 할 축이 있는지 URDF 및 실제 모델 의미를 검토한다. 근거 없이 `continuous`로 바꾸지 않는다.
5. SRDF self-collision matrix는 Setup Assistant에서 생성하되 자동 생성 결과를 확인한다. 인접 링크의 정상 접촉을 제외하는 규칙이 실제 기구와 일치하는지 검토하고, 공·바구니 충돌은 별도 Planning Scene 대상으로 남긴다.

## 4. 월드 작업 목표 기준

고정 목표는 Gazebo 월드 SDF의 pose를 기준으로 한다(`my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf` 62–77행).

| 물체 | 월드 좌표 기준값 | 설계 처리 |
|---|---|---|
| 로봇 | `(0, 0, 0, 0, 0, 0)` | `base_link`와 월드 기준 관계 확인 |
| 공 | `(0.55, -0.25, 0.0325, 0, 0, 0)` | 고정 월드 목표; 공 중심 좌표와 충돌 반경/형상 확인 |
| 바구니 | `(0.75, 0.25, 0.000022, 0, 0, 0)` | 고정 월드 목표; 바구니의 실제 내부/입구 중심과 SDF 모델 origin 차이 확인 |

수치는 SDF pose 그대로 기록한다. 공의 캐시된 모델 SDF에서 collision geometry는 반지름 0.0325 m 구이며, 중심 pose z=0.0325 m와 일치한다. 바구니 모델은 link origin 기준 visual/collision 모두 동일한 `meshes/model.obj`를 사용한다. OBJ vertex bounding box는 x `[-0.119587, 0.119698]`, y `[-0.080868, 0.081326]`, z `[-0.000022, 0.147591]` m이고, SDF pose를 더하면 바닥은 월드 z≈0, 최고점은 z≈0.147613 m다. bbox의 x/y 중심은 원점에 매우 가깝다. 하지만 mesh 전체 bbox는 입구 안쪽 경계/유효 낙하 영역을 정의하지 않으므로, 놓기 성공 판정과 충돌 형상 근사는 후속 단계에서 따로 정한다. 공 접근·놓기 end-effector 목표 pose의 자세와 높이는 2단계 도달성 검증에서 확정한다.

## 5. 수행 절차

1. **환경 버전 기록**: 새 로그인 셸에서 `ROS_DISTRO`, `GZ_VERSION`, `gz sim --versions`, 관련 `dpkg-query` 결과를 수집한다. ROS Jazzy, Gazebo Harmonic, MoveIt 2, `ros_gz`, `gz_ros2_control`의 정확한 패키지 버전과 OS 버전을 결과 문서에 기록한다.
2. **공식 지원 확인**: ROS–Gazebo 공식 조합표에서 Jazzy/Harmonic을 확인하고, MoveIt 공식 설치 문서에서 Jazzy 지원과 바이너리 패키지 경로를 확인한다. 저장소 README 및 기존 ros2_control 단계의 기준과 불일치가 있으면 원인과 선택을 기록한다.
3. **MoveIt 설치 상태 확인**: `moveit_configs_utils`, `moveit_setup_assistant`, `move_group`, `moveit_planners_ompl`, `moveit_simple_controller_manager`의 ROS package prefix를 확인한다. 빠진 패키지는 설치 상태만 보고하고, 이 단계에서 시스템 패키지 설치나 저장소 의존성 변경은 하지 않는다.
4. **URDF 기준 검사**: Xacro를 확장하고 link/joint/group 후보 및 한계를 표로 만든다. 기존 모델 일관성 검사를 사용해 SDF와 대조한다. 실패 항목은 원인, 해당 파일/요소, 후속 수정 필요 여부를 명시한다.
5. **프레임 관계 검사**: 모델을 robot_state_publisher에 공급해 TF tree를 확인하거나 동등한 URDF 변환 계산을 수행한다. `base_link`와 `tool0`의 계획 프레임/종단 프레임 후보를 정하고, 그리퍼 중심 오프셋을 함께 기록한다.
6. **월드 물체 기준 검사**: SDF에서 공·바구니 포함 모델의 pose 및 충돌 형상을 확인하고, SDF pose와 실제 기준점의 차이를 구분한다.
7. **판정 및 인계**: 호환 조합, 그룹·프레임 기준, 확인된 모델 문제, 미해결 항목을 아래 산출물 형식으로 정리한다. 코드·Xacro·launch·패키지 파일은 수정하지 않는다.

## 6. 산출물

이 단계의 결과 기록은 이 문서의 `## 7. 조사 결과 및 판정` 아래에 작성한다.

- 사용 OS/ROS/Gazebo/MoveIt/ros_gz/gz_ros2_control 버전과 공식 지원 근거 링크
- MoveIt 패키지 설치 상태 및 빠진 의존 패키지 목록
- 확정 planning group, 기본 자세의 출처, base/EEF/gripper 프레임 관계
- 8개 관절의 유효 limit 및 SDF/Xacro 비교 결과
- 충돌 geometry 로딩 및 self-collision matrix 검토 결과
- 공·바구니 기준 좌표와 SDF origin에서 작업 기준점까지 필요한 변환
- 단계 2에서 반영할 모델 수정·설정 작업 및 미해결 이슈

## 7. 조사 결과 및 판정 (2026-10-12)

### 확인된 환경

- OS는 Ubuntu 24.04.5 Noble이다. ROS 2 Jazzy, Gazebo Harmonic (`gz-sim` 8.15.0), `ros_gz` 1.0.24, `gz_ros2_control` 1.2.20이 설치되어 있다.
- MoveIt 2 Jazzy 패키지(core, configs utils, kinematics, OMPL planner, planning, RViz, Setup Assistant, simple controller manager 등) 2.12.4 계열 및 `moveit_ros_move_group`이 설치되어 있다. 확인한 ROS package prefix는 `/opt/ros/jazzy`다.
- 공식 Gazebo ROS 호환표는 Jazzy/Harmonic을 권장 조합으로 표시하고 MoveIt 문서는 Jazzy를 지원한다. 따라서 기존 조합을 유지한다. 확인한 버전은 2026-10-12 로컬 apt 설치값이다.
- `my_arm_with_ball_description`을 symlink install로 빌드한 뒤 overlay한 환경에서 MoveIt Setup Assistant를 `--debug --urdf_path`로 실행했다. URDF가 `Loaded ur5_rg2 robot model` 및 `Loaded robot model` 로그로 로드됐고 collision mesh를 해석하는 오류 없이 완료됐다. SRDF가 아직 없는 시점의 `No root/virtual joint specified in SRDF. Assuming fixed joint` 로그는 Xacro root `world`의 고정 베이스 정의와 일치한다.
- `check_urdf`에서 root `world`와 전체 link/joint tree를 파싱했다. 기존 `test_model_consistency.py`에서 4개 검사가 모두 통과했다. description 패키지 빌드도 성공했다.
- 헤드리스 시뮬레이션에서 broadcaster `/joint_states`를 확인했다. 여섯 팔 관절은 0 rad에 수치 오차 내로 있고 두 finger joint도 약 0 rad다. TF에서 `world → base_link`는 항등이며 fixed TF `/tf_static`은 `wrist_3_link → tool0` 및 `wrist_3_link → rg2_hand`를 발행한다.
- 로컬 Gazebo Fuel cache의 공 SDF 및 바구니 SDF/OBJ를 확인했다. 공 collision은 반지름 0.0325 m 구다. 바구니 visual/collision은 동일한 OBJ mesh를 사용하고, mesh AABB와 SDF pose는 위 표에 반영했다. 입구 안쪽 유효 영역은 AABB만으로 확정할 수 없다.

### 1단계 설계 판정

- 호환 조합: **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic + Jazzy MoveIt 2 + Jazzy ros_gz/gz_ros2_control**.
- planning group: `ur5` 6축 팔 그룹과 `gripper` 2축 손가락 그룹. 모델에 두 joint group 모두 구성 가능한 모든 joint가 존재한다. 팔 group의 chain tip/EEF 링크는 `rg2_hand`로 하고 `tool0`는 보조 프레임으로 둔다.
- MoveIt planning frame은 URDF root `world`; manipulator chain base는 `base_link`다. 두 link 간 고정 변환은 항등이다. 별도 SRDF virtual joint는 추가하지 않는다.
- `home` 명목 자세는 8개 제어 관절이 0 rad인 상태다. 실제 시작 상태는 `/joint_states`에서 소수점 오차만 확인했다.
- 모델 소스: Gazebo 물리 시뮬레이션은 현재 SDF, MoveIt의 kinematic robot model은 Xacro가 생성하는 URDF. 어느 한쪽을 다른 쪽으로 대체하지 않는다.
- 월드 목표: 공·바구니의 SDF pose를 기준으로 삼되, 바구니 입구 중심 및 충돌형상 기준점은 포함 SDF 모델 확인 후 분리 기록한다.
- SRDF 생성, 그룹/EEF 등록, self-collision matrix 생성 및 자동 생성 결과 검토는 다음 설정 작업에서 한다. MoveIt Setup Assistant가 모델 로드 시 fixed root를 가정하는 것과 모델 collision mesh 로딩은 확인했다.
- 바구니의 정확한 입구 안쪽 형상은 mesh AABB 조사로 확정되지 않았다. 이 값은 단계 2의 Planning Scene 기하 근사 및 단계 6의 놓기 판정에서 추가 조사한다.

## 8. 완료 기준

- [x] OS와 ROS/Gazebo/MoveIt/ros_gz 관련 설치 버전이 명령 출력으로 확인·기록됐다.
- [x] 공식 지원 자료로 현재 조합이 지원됨을 확인했고, 기존 조합 유지 근거를 기록했다.
- [x] `ur5` 및 `gripper` 그룹의 관절 후보가 URDF에 존재하며, 시뮬레이션 초기 `/joint_states`에서 명목 `home` 수치가 확인됐다.
- [x] `base_link`, `tool0`, `rg2_hand`, `world`의 연결과 변환을 확인하고 planning frame 및 EEF 기준을 확정했다.
- [x] 팔 6축·finger 2축의 한계를 SDF/Xacro consistency 검사로 확인했고, collision mesh 로딩을 MoveIt Setup Assistant에서 확인했다.
- [x] 공·바구니의 월드 pose와 충돌 형상을 조사해 기록했다.
- [x] MoveIt Setup Assistant가 URDF와 package mesh URI를 로드했다. 설명 패키지 설치를 위해 `colcon build --packages-select my_arm_with_ball_description --symlink-install`도 통과했다.
- [x] 애플리케이션·Xacro·launch 코드는 수정하지 않았다. 결과 기록은 이 계획 문서와 상위 계획 문서만 갱신한다.
- [ ] SRDF 자체 생성 및 self-collision matrix 내용 검토는 아직 하지 않았다. `moveit_apply.md` 단계 2 설정 작업으로 넘긴다.
- [ ] 바구니 유효 내부 경계는 mesh AABB만 확인했으므로, Planning Scene 및 놓기 판정에 사용할 안쪽 치수는 단계 2/6에서 추가 확인한다.

## 9. 예상 위험과 대응

| 위험 | 영향 | 대응 |
|---|---|---|
| Xacro와 SDF가 독립적으로 유지되어 차이가 생김 | MoveIt collision/kinematic 모델과 Gazebo 물리가 다르게 보일 수 있음 | 기존 consistency test 결과를 근거로 두 모델을 비교하고 불일치는 수정 후보로 명시 |
| URDF의 `world` 고정 관절과 MoveIt planning frame/virtual joint를 중복 설정 | 모델 로드 오류 또는 예상하지 못한 planning frame | SRDF virtual joint 추가 전 URDF와 TF 구성을 확인하고 하나의 고정 베이스 모델만 사용 |
| `tool0`가 실제 그리퍼 작업 중심과 오프셋됨 | 공 접근/놓기 포즈가 틀어짐 | `tool0`와 `rg2_hand` transform 및 공 접촉면 기준점을 함께 기록하고 task pose 변환을 명시 |
| 바구니 모델 origin이 입구 중심과 다름 | 충돌 장면 및 놓기 목표가 어긋남 | SDF 내부 링크/충돌 geometry를 읽어 입구 기준점을 별도 정의 |
| mesh 충돌 형상이 과도하게 복잡하거나 파서 호환 문제가 있음 | 계획 지연 또는 모델 로드 실패 | 원인과 구체 link를 기록하고 2단계에서 단순 collision geometry 여부를 결정 |
| 패키지 버전이 시스템 업데이트로 바뀜 | 재현성 저하 | 결과에 확인 날짜와 전체 패키지 버전을 기록하고 이후 구현 검증 시 재확인 |
