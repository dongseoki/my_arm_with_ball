# MoveIt 사용 방식 검토: `grasp_test.py`와 MTC 예제

## 질문과 결론

`grasp_test.py`에서 `/compute_ik` 같은 ROS 서비스를 직접 호출하는 방식이 일반적인지, `moveit_guide/src/mtc_tutorial/src/mtc_node.cpp`의 MoveIt Task Constructor(MTC) 방식과 비교했다.

**결론부터 말하면, 둘 다 정식 MoveIt 사용 방식이지만 같은 계층의 대안은 아니다.** 보통의 애플리케이션에서는 단일 목표를 주고 계획·실행할 때 `MoveGroupInterface`(C++) 또는 `moveit_py`(Python) 같은 상위 API를 쓰는 편이 자연스럽다. `/compute_ik` 직접 호출은 유효한 저수준 기능이지만, IK만 계산하며 경로 계획이나 로봇 실행은 하지 않는다. 반면 MTC는 접근·집기·부착·들기·이동·놓기처럼 서로 연결된 여러 단계를 작업 단위로 구성하고 계획하는 프레임워크다. MTC가 모든 MoveIt 작업의 기본이거나 유일한 “일반 방식”이라는 뜻은 아니다.

## 이 코드들이 MoveIt을 사용하는 방식

| 비교 항목 | `grasp_test.py` | `mtc_node.cpp` |
|---|---|---|
| 추상화 수준 | ROS 2 액션·서비스 클라이언트를 시험 노드에서 직접 조합한다. | 여러 MoveIt 동작을 MTC stage와 container로 구성한 task로 표현한다. |
| 팔·그리퍼 이동 | `/move_action`의 `MoveGroup` action에 관절 제약 목표를 보내 계획·실행한다. | `MoveTo`, `MoveRelative`, `Connect` stage와 각기 다른 solver로 이동 단계를 만든 뒤 `task_.plan()` / `task_.execute()`를 사용한다. |
| IK/FK | 고정된 손 pose마다 `/compute_ik` 서비스를 호출해 충돌 회피 IK 해를 찾고, `/compute_fk`로 현재 `rg2_hand` pose를 구한다. IK 결과만으로 움직이지 않고 별도 MoveGroup action으로 이동시킨다. | `GenerateGraspPose`에 `ComputeIK` wrapper를 붙여 후보 grasp pose의 IK를 task 계획 안에서 푼다. 이것은 사용자 노드가 `/compute_ik` 서비스를 직접 호출하는 것과 같은 사용법이 아니다. |
| Planning Scene | `ApplyPlanningScene` / `GetPlanningScene`으로 Gazebo 부착·분리 뒤 장면을 변경하고 상태를 확인한다. | `ModifyPlanningScene` stage의 `attachObject` / `detachObject`로 task의 계획 상태를 단계별로 바꾼다. |
| 작업 흐름 | Python 절차 코드가 실제 접촉 메시지를 기다리고, 닫기 action을 취소·종료한 뒤 Gazebo 부착 서비스를 호출한다. 그 다음 MoveIt 장면을 동기화한다. | 현재 상태 → 손 열기 → 접근 및 grasp 후보 → 닫기 → 계획 장면 부착 → lift → place → 열기·분리 → retreat/home을 하나의 연계된 계획 task로 만든다. |

근거 파일 위치:

- `my_arm_with_ball_application/scripts/grasp_test.py:118-167` — `/move_action`용 목표 구성·계획/실행 결과와 관절 도달 상태 확인.
- `my_arm_with_ball_application/scripts/grasp_test.py:169-204` — 접촉을 기다려 그리퍼 목표를 취소하고 종료 결과와 5° 조건을 확인한 뒤 Gazebo attach 요청.
- `my_arm_with_ball_application/scripts/grasp_test.py:206-249` — `/compute_fk`와 `/compute_ik` 직접 호출, IK 후보를 joint goal로 실행.
- `my_arm_with_ball_application/scripts/grasp_scene.py:87-114, 200-242` — 부착/분리/위치 갱신 요청과 Planning Scene 상태 검증.
- `moveit_guide/src/mtc_tutorial/src/mtc_node.cpp:68-97, 99-239, 242-333` — MTC task 생성·계획·실행, pick/lift/place/attach/detach 단계.

## 어느 쪽이 더 일반적인가?

“일반적인”을 **대부분의 MoveIt 프로그램이 처음 사용하는 상위 API**라는 뜻으로 보면, 단일 이동은 `MoveGroupInterface` 또는 `moveit_py`가 더 대표적인 출발점이다. 공식 `MoveGroupInterface` 문서는 이를 가장 간단한 사용자 인터페이스로 설명하고 관절·pose goal 지정, 계획·이동, 물체 추가 및 부착/분리를 제공한다. 공식 Python API도 pose goal이나 joint goal을 설정해 계획·실행할 수 있다. 따라서 **Python에서 MoveIt을 쓰려면 `/compute_ik` 서비스를 반드시 직접 불러야 하는 것은 아니다.**

“일반적인”을 **다단계 조작 작업을 구조화해 전체 계획을 만든다**는 뜻으로 보면, MTC가 이에 더 맞는 전문 도구다. 공식 MTC 설명은 상호 의존적인 하위 작업을 구성하는 프레임워크로 정의하고, 공식 pick-and-place 예제가 grasp pose 생성, `ComputeIK`, scene collision 변경, attach, lift 및 place를 stage로 표현한다. 즉 MTC 예제는 더 높은 작업 수준에서 같은 MoveIt 계획 기능을 조합한다.

`/compute_ik`는 그 자체로 비표준이거나 잘못된 인터페이스가 아니다. 다만 이 서비스의 역할은 주어진 pose와 seed 등에 맞는 관절 해를 반환하는 것이며, 충돌 없는 전체 경로를 계획하거나 controller로 실행해 주는 것은 아니다. 그래서 현재 구현처럼 여러 seed로 IK를 반복하고, 결과를 joint constraint 목표로 다시 만들어 MoveGroup action에 보내는 구성은 **가능하지만 더 저수준이고, 단일 pose goal용 상위 MoveIt API보다 애플리케이션 코드가 더 많은 일을 직접 맡는 방식**이다.

## 왜 4단계 시험은 이렇게 구현했나?

이 구현은 최종 Pick-and-Place 애플리케이션의 표준 구조를 제시하려는 것이 아니라, **4단계의 제한된 통합 시험**을 빠르게 재현하려는 선택이었다. 시험의 핵심 조건은 사전에 정해 둔 접촉 상태를 가정하는 게 아니라 Gazebo가 실제 손가락-공 접촉을 보고한 뒤에 닫기 동작을 멈추고, 측정 관절값을 확인한 다음 물리 공을 부착하는 것이다. 따라서 절차가 `MoveIt 이동 → Gazebo 접촉 이벤트 대기/닫기 취소 → 조건 판정 → Gazebo attach → MoveIt Planning Scene 동기화`로 나뉘었다.

당시 선택 이유에 대한 설명은 구현 당시 남긴 별도 의사결정 기록이 아니라 현재 코드와 4단계 범위에 근거한 회고다. 제한된 시험 스크립트에서 이미 실행 중인 `/move_action`, IK/FK, Planning Scene 및 새 Gazebo service 경계를 직접 연결하면 실제 시뮬레이션 결과를 단계별로 검증하기 쉽고, 아직 5단계 전체 작업 planner나 MTC 구성을 추가하지 않아도 된다. 고정된 손 목표 pose를 여러 seed로 시도하는 `/compute_ik` 사용도 시험 중 해당 pose의 충돌 회피 해를 찾기 위한 구체적 구현 선택이다. 그렇다고 이 구조를 일반 애플리케이션에서 가장 권장되는 패턴이라고 주장하는 것은 아니다.

또 하나 중요한 차이는 **계획 장면의 가상 부착과 Gazebo의 물리 부착이 서로 다른 일**이라는 점이다. `mtc_node.cpp`의 `ModifyPlanningScene::attachObject()`는 MTC task의 계획 상태에 object를 attached로 표현한다. 이 코드에는 Gazebo 접촉 상태를 기다리거나 실제 Gazebo entity를 부착하는 연동은 보이지 않는다. 현재 4단계 구현은 Gazebo에서 실제 부착/분리를 먼저 요청하고 성공을 확인한 다음 MoveIt Planning Scene을 갱신한다. 향후 MTC를 사용하더라도 이 물리 검증·연동은 별도로 구현해야 한다.

## 판단 및 다음 단계 권고

- **4단계 기능 시험:** 현재 구조는 목적에 맞다. 실제 접촉 여부와 실행 중 action 취소처럼 런타임에 관찰해야 하는 사건을 시험 코드가 명시적으로 확인하고 있어 시험 실패 지점을 분리해 보기 쉽다.
- **코드의 추상화 측면:** `/compute_ik`를 매 pose마다 직접 호출하는 것이 “더 일반적”인 MoveIt 패턴은 아니다. 상위 pose-goal API로 같은 동작을 표현할 수 있다. 현재 코드는 저수준 API 사용의 장단점을 알고 선택한 시험용 구현으로 이해하는 편이 정확하다.
- **5단계 전체 작업:** 후보 grasp/place와 접근·닫기·들기·이동·놓기 단계를 함께 비교·계획해야 한다면 MTC는 유력한 선택이다. 실제 접촉이 기대와 다를 때 중단하거나 재계획하는 런타임 상태/검증 경계는 MTC task와 별도로 분명히 둬야 한다. 단순 순차 작업이면 MoveGroupInterface/MoveItPy를 이용한 명시적 상태 기계도 대안이다.
- 따라서 현 시점 결론은 **“MTC가 보편적으로 더 낫다”가 아니라, 현재 4단계 테스트에는 이벤트 중심 절차 코드가 타당하고, 5단계의 여러 계획 단계를 구성할 때 MTC가 더 잘 맞을 가능성이 크다”**이다. 구현 전 해당 워크스페이스의 MoveIt/MTC 버전과 설치·실행 의존성은 별도로 확인해야 한다.

## 공식 참고 자료

- [Move Group C++ Interface](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html) — MoveGroupInterface의 상위 목표 지정·계획·실행 기능.
- [Motion Planning Python API (`moveit_py`)](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html) — Python에서 pose/joint goal 계획과 실행.
- [MoveIt Task Constructor 개념](https://moveit.picknik.ai/main/doc/concepts/moveit_task_constructor/moveit_task_constructor.html) — MTC가 상호 의존 하위 작업을 조합하는 task framework임을 설명.
- [MTC Pick-and-Place 튜토리얼](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) — `ComputeIK`, scene attach/detach, 접근·lift 등 stage 예제.
- [MoveGroup 노드](https://moveit.picknik.ai/main/doc/concepts/move_group.html) 및 [MoveIt `GetPositionIK` 메시지 정의](https://github.com/moveit/moveit_msgs/blob/6066314bc50472c23a0712f317396bd120a07227/srv/GetPositionIK.srv) — ROS 인터페이스와 IK 응답의 범위.

공식 웹 문서는 확인일 기준 MoveIt `main`/Rolling 문서다. 프로젝트의 설치판은 ROS 2 Jazzy이므로 개념 비교에 사용했으며, 문서에 있는 API 예제를 이 프로젝트에서 곧바로 빌드할 수 있다고 가정하지 않는다.
