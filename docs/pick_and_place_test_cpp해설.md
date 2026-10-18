# pick_and_place_test.cpp 코드 해설

## 1. 읽는 방법

이 문서는 **실행 구조 → 상태 전이 → 단계별 함수 → 공통 도우미** 순서로 코드를 따라 읽기 위한 안내서다. 전체 작업은 C++ 상태 머신이 조율하고, 이동 구간별로 MTC Task를 생성한다. 하나의 MTC Task가 Gazebo 확인까지 모두 실행하는 구조는 아니다.

- 분석 대상: [pick_and_place_test.cpp L1–L802](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L1-L802)
- 링크 형식: [마크다운테스트.md](마크다운테스트.md)의 `상대경로#L시작-L끝` 형식을 사용한다.
- 행 번호는 문서 작성 시점의 소스 기준이다. 코드 변경 시 함께 갱신해야 한다. 뷰어가 행 앵커를 지원하지 않으면 표에 적힌 함수명과 행 번호로 찾아간다.
- 흐름도는 Mermaid 지원 뷰어에서 표시된다. 도형에 적힌 행 번호의 실제 이동 링크는 바로 아래 표에서 제공한다.

추천 읽기 순서:

1. [main() L790–L802](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L790-L802): 실행 시작점.
2. [생성자 L58–L149](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L58-L149): 파라미터, 구독, 클라이언트, 타이머 등록.
3. [tick() L689–L758](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L689-L758): 전체 흐름의 중심.
4. 아래 단계 연결표에서 관심 있는 함수로 이동.
5. 서비스·액션·MTC 공통 도우미를 확인.

## 2. 전체 실행 구조: 누가 무엇을 실행하는가

```mermaid
flowchart TD
    MAIN["main: 노드 생성과 spin / L790–802"] --> INIT["생성자: 구독과 타이머 등록 / L58–149"]
    INIT --> EXEC["ROS executor"]
    EXEC --> SUB["구독 콜백: 관측 데이터 갱신 / L99–136"]
    EXEC --> TICK["100ms wall timer: tick / L689–758"]
    EXEC --> RESP["서비스와 Action 응답 처리"]
    SUB --> DATA["joints_, pose_, contact_, attached_, stable_"]
    DATA --> TICK
    TICK -->|"진행 중 작업이 없으면 시작"| ASYNC["std::async 작업 스레드 / L738–757"]
    ASYNC --> WORK["계획, 실행, 서비스 요청, 결과 대기"]
    RESP --> WORK
    WORK --> FUTURE["work_ future: 결과 또는 예외"]
    FUTURE -->|"다음 tick에서 완료 여부 확인"| TICK
```

| 구성 | 읽을 코드 | 역할 |
| --- | --- | --- |
| 시작점 | [main() L790–L802](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L790-L802) | 노드를 만들고 `MultiThreadedExecutor::spin()`으로 ROS 콜백을 처리한다. |
| 관측 수집 | [구독 콜백 L99–L136](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L99-L136) | 관절, 접촉, 부착, 공 pose와 수신 시각을 저장한다. 공 pose 콜백은 놓기 판정도 갱신한다. |
| 주기 실행 | [타이머 등록 L147](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L147) | `create_wall_timer(100ms, ...)`로 `tick()`을 호출한다. |
| 상태 관리 | [tick() L689–L735](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L689-L735) | 작업 결과를 확인하고 다음 상태 또는 실패 대기로 전환한다. |
| 실제 작업 | [작업 분기 L736–L757](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L736-L757) | 현재 상태를 복사한 `selected`에 따라 별도 스레드에서 해당 함수를 실행한다. |
| 공유 데이터 보호 | [멤버 L760–L787](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L760-L787) | `mutex_`, `planning_mutex_`, `stopping_`, `work_` 등이 콜백과 작업 스레드를 연결한다. |

### 중요한 구분

- `tick()`은 `work_.wait_for(0ms)`로 **완료 여부만 확인**한다. 작업이 끝나지 않았다면 바로 반환한다.
- 실제 작업 스레드에는 `await()`의 future 대기와 접촉·부착·관절 확인 루프가 있다. **타이머 기반이라고 모든 함수가 비블로킹인 것은 아니다.** 블로킹 작업을 executor 콜백 밖으로 분리한 구조다.
- 이 파일은 별도 callback group을 지정하지 않는다. `MultiThreadedExecutor`가 있다는 사실만으로 이 노드의 기본 그룹 콜백들이 병렬 실행된다고 읽으면 안 된다. 작업과 ROS 콜백의 동시 진행에서 핵심은 `std::async` 분리다.
- `mutex_`는 콜백과 작업 스레드의 관측 데이터 접근을 보호하고, `planning_mutex_`는 종료 시 사용할 `planning_task_` 포인터를 보호한다.

## 3. 작업 상태 전이도

```mermaid
flowchart TD
    W["WAIT_READY: 준비 확인 / L288–317"] -->|"준비 완료"| A["APPROACH: 열기와 접근 / L560–568"]
    W -->|"미준비, 제한 시간 이내"| W
    A --> C["CLOSE: 접촉 닫기와 Gazebo 부착 / L569–621"]
    C --> AT["ATTACH: Planning Scene 부착 / L745"]
    AT --> T["TRANSPORT: lift와 운반 / L622–655"]
    T --> R["RELEASE: 열기와 분리 / L656–662"]
    R --> V["VERIFY_PLACE: 안정 판정 / L721–735"]
    V -->|"아직 안정되지 않음, 제한 시간 이내"| V
    V -->|"안정 성공"| F["REFRESH_SCENE: 최종 공 위치 반영 / L748"]
    F --> B["RETURN: 후퇴와 복귀 / L749–752"]
    B --> S["SUCCEEDED_WAITING: 계속 대기 / L714"]
    ERR["작업 예외 또는 준비·놓기 시간 초과"] --> FAIL["FAILED_WAITING: ERROR 후 대기 / L679–683"]
    W -.-> ERR
    A -.-> ERR
    C -.-> ERR
    AT -.-> ERR
    T -.-> ERR
    R -.-> ERR
    V -.-> ERR
    F -.-> ERR
    B -.-> ERR
```

상태 정의는 [L164–L165](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L164-L165), 로그용 이름은 [stateName() L663–L678](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L663-L678)에 있다. `CLOSE`는 로그에서 `CLOSE_CONTACT`, `ATTACH`는 `ATTACH_SCENE`으로 출력된다.

### 단계별 코드 연결표

| 단계 | 실제 호출 경로와 코드 | 하는 일 및 다음 단계 조건 |
| --- | --- | --- |
| 준비 | [tick → readiness() L739–L740](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L739-L740), [함수 L288–L317](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L288-L317) | 최신 관측, 3개 active 컨트롤러, Action/서비스, 장면의 apple/basket을 확인한다. 빈 문자열이면 준비 완료. |
| 접근 | [approach() L560–L568](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L560-L568) | 접촉 충돌 허용 → 그리퍼 열기 → 시작 관절 → pregrasp 관절 → grasp pose를 MTC로 계획·실행한다. |
| 접촉 닫기 | [closeContact() L569–L621](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L569-L621) | `/move_action`으로 그리퍼를 닫고 접촉을 기다린다. 접촉 후 필요 시 goal 취소, 종료 결과와 손가락 개방량을 확인한 뒤 `trigger(true)`로 Gazebo 부착을 요청한다. |
| 장면 부착 | [ATTACH 분기 L745](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L745), [syncScene() L394–L429](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L394-L429) | Gazebo 부착 상태 확인 후 MoveIt의 공을 손에 부착된 물체로 반영하고 조회 검증한다. **Gazebo 부착 요청은 앞 단계에서 이미 수행한다.** |
| 운반 | [transport() L622–L655](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L622-L655) | 수직 lift MTC Task → 부착 확인 → 손/공 XY 오프셋 보정 → 바구니 위 이동 MTC Task → 부착 확인. |
| 놓기 | [release() L656–L662](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L656-L662) | MTC로 손가락을 연 뒤 Gazebo 분리를 요청하고 MoveIt 장면의 공을 월드 물체로 바꾼다. |
| 안정 판정 | [pose 콜백 L119–L136](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L119-L136), [tick 판정 L721–L735](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L721-L735) | 콜백이 `stable_`을 계산한다. 타이머는 성공·대기·시간 초과를 결정한다. 이 상태에서는 새 작업 스레드를 반복 생성하지 않는다. |
| 장면 재갱신 | [REFRESH_SCENE 분기 L748](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L748) | 공이 분리된 상태인지 확인하고 정착한 공의 최신 pose를 `syncScene(false, true)`로 반영한다. |
| 후퇴·복귀 | [RETURN 분기 L749–L752](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L749-L752) | retreat pose → 저장한 시작 관절로 복귀하는 MTC Task를 실행하고 손과 공의 접촉 허용을 해제한다. |
| 성공·실패 대기 | [fail() L679–L683](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L679-L683), [종료 상태 반환 L693](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L693), [성공 전환 L714](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L714) | 타이머는 바로 반환한다. 자동 재실행이나 정상 작업 실패에 따른 노드 종료는 하지 않는다. |

## 4. tick()을 읽는 순서

```mermaid
flowchart TD
    ENTER["tick 진입: mutex 잠금 / L692"] --> END{"성공 또는 실패 대기 상태? / L693"}
    END -->|예| RET["이번 tick 반환"]
    END -->|아니오| HAS{"work_가 존재하는가? / L694"}
    HAS -->|예| DONE{"작업 완료? / L695"}
    DONE -->|아니오| RET
    DONE -->|예| GET["work_.get: 결과 또는 예외 / L697"]
    GET -->|예외| FAIL["fail 후 반환 / L718"]
    GET --> NEXT["준비 결과 처리 또는 다음 상태 전환 / L698–717"]
    NEXT --> CHECK["준비 timeout 및 놓기 안정 판정 / L720–735"]
    HAS -->|아니오| CHECK
    CHECK -->|"미완료 대기 또는 실패"| RET
    CHECK -->|"다음 동작 필요"| START["selected 상태의 비동기 작업 시작 / L736–757"]
```

위 그림은 제어 흐름 요약이다. 정확한 분기는 [tick() 전체 L689–L758](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L689-L758)를 참고한다. 특히 준비 결과가 아직 미충족이면 L700–L702에서 반환하고 다음 tick에서 다시 준비 작업을 시작한다.

`work_`가 진행 중이면 새 작업을 시작하지 않으므로 작업 단계가 중복 실행되지 않는다. 작업 함수가 정상 반환한 뒤에야 L707–L715에서 다음 상태로 넘어간다. `VERIFY_PLACE`는 예외적으로 구독 콜백이 만든 판정값을 타이머가 직접 처리한다.

## 5. MTC는 어디에 적용되는가

| 함수 | MTC 구성 | 코드 |
| --- | --- | --- |
| `configureTask()` | 로봇 모델과 현재 Planning Scene을 읽고 `FixedState`를 시작 상태로 추가한다. | [L523–L531](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L523-L531) |
| `addJointMove()` | `PipelinePlanner`와 `MoveTo`를 사용한다. 관절 이름과 목표값을 대응시킨다. | [L532–L543](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L532-L543) |
| `addPoseMove()` | `GeneratePose`를 `ComputeIK`로 감싸고 `Connect`와 연결한다. 목표 pose의 IK 해와 연결 경로를 찾는다. | [L544–L559](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L544-L559) |
| `transport()`의 lift | `CartesianPath`와 `MoveRelative`로 월드 +Z 방향 수직 이동을 만든다. | [L625–L642](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L625-L642) |
| `executeTask()` | 초기화 → `task.plan(3)` → 첫 solution을 메시지로 변환 → 컨트롤러 지정 → `/execute_task_solution`으로 실행 → 결과 및 관절 도달 확인. | [L468–L522](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L468-L522) |

```mermaid
flowchart LR
    SCENE["현재 장면: configureTask"] --> STAGES["이동 Stage 추가"]
    STAGES --> PLAN["executeTask: task.plan"]
    PLAN --> SOL["solution 메시지"]
    SOL --> ACTION["execute_task_solution Action"]
    ACTION --> RESULT["실행 성공 확인"]
    RESULT --> JOINT["verifyJoints: 실제 관절 확인"]
```

**목표와 경로는 다르다.** [설정 YAML](../my_arm_with_ball_application/config/pick_and_place_test.yaml)의 pregrasp 관절과 grasp/place/retreat pose는 목표값이다. 그 사이 궤적은 계획 시 생성된다. `lift_pose`는 현재 구현에서 Z만 사용하며, `place_pose`의 XY는 측정된 공-손 차이를 빼서 보정한다.

그리퍼 접촉 닫기는 MTC Task가 아니라 [closeContact()의 MoveGroup Action L571–L586](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L571-L586)을 사용한다. 접촉에 반응해 실행을 취소해야 하는 구간이므로 다른 이동 구간과 읽는 경로가 다르다.

## 6. Gazebo 물리 상태와 MoveIt 장면은 별개다

| 책임 | 함수와 코드 | 확인 내용 |
| --- | --- | --- |
| 물리 부착·분리 | [trigger() L435–L449](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L435-L449) | `/grasp/attach` 또는 `/grasp/detach` 서비스 성공 뒤 `/grasp/attached` 관측이 요청한 값인지 기다린다. |
| 부착 유지 확인 | [requireAttachment() L430–L434](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L430-L434) | 부착 관측의 최신성과 값을 확인한다. 실제 부착 동작을 실행하는 함수는 아니다. |
| 장면 변경 | [syncScene() L394–L429](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L394-L429) | 부착 시 공 pose를 손 좌표계로 변환하고, 분리 시 월드 pose로 반영한다. |
| 장면 검증 | [verifyScene() L354–L393](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L354-L393) | 공의 world/attached 소속, 구 형상·반지름·좌표계 및 위치 오차를 검증한다. |
| 접촉 허용 | [touchAllowed() L334–L353](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L334-L353) | Planning Scene의 공과 손 링크 사이 충돌 허용을 바꾼다. Gazebo의 물리 충돌을 끄는 함수가 아니다. 공과 pick_area_floor의 허용은 true로 둔다. |

추적 예: **접촉 확인 → `trigger(true)` → Gazebo 부착 관측 → ATTACH 상태 → `syncScene(true)` → `verifyScene()` → 운반**.

## 7. 놓기 판정과 시간 기준

판정 구현은 별도 헤더의 [PlaceStability::update() L17–L45](../my_arm_with_ball_application/src/place_stability.hpp#L17-L45)에 있다.

1. 공 중심이 `basket_bounds`의 X/Y/Z 범위 안인지 확인한다.
2. 연속 공 위치의 이동 거리 ÷ 메시지 timestamp 차이로 속력을 추정한다.
3. 내부 위치와 저속 조건을 연속 만족한 시간이 `stable_duration` 이상이면 성공이다.
4. 위치·속도 조건이 깨지면 안정 시간 누적을 초기화한다. 첫 표본만으로는 속력을 알 수 없어 성공하지 않는다.
5. 타이머는 관측이 오래됐으면 판정을 초기화하고, `place_timeout` 초과 시 실패로 바꾼다.

| 시간 용도 | 실제 기준 | 코드 |
| --- | --- | --- |
| 타이머 호출 | wall timer, 100ms | [L147](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L147) |
| 준비/놓기 제한시간, 수신 최신성, 자체 대기 | `std::chrono::steady_clock` | [Clock 별칭 L48](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L48), [secondsSince/fresh L186–L189](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L186-L189) |
| 공 속력과 연속 안정 시간 | 공 pose 메시지의 stamp | [L132–L134](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L132-L134) |

따라서 `use_sim_time=true`라고 해서 이 파일의 모든 제한시간이 시뮬레이션 시간으로 바뀌는 것은 아니다. 시뮬레이션을 멈춰도 wall timer와 steady-clock 기반 제한시간은 계속 진행한다.

## 8. 공통 도우미: 필요한 때 찾아보기

| 함수 | 역할 및 주의점 | 코드 |
| --- | --- | --- |
| `parameter()`, `poseParameter()` | 설정 읽기, pose 배열 검사, quaternion 정규화. YAML override가 있으면 C++ 기본값보다 우선한다. | [L166–L185](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L166-L185) |
| `measured()`, `ball()` | 최신 관절 또는 공 pose를 잠금 아래 복사해 가져온다. 오래된 값이면 예외. | [L190–L207](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L190-L207) |
| `await()` | 작업 스레드에서 future를 50ms씩 기다리고 종료·timeout을 확인한다. ROS 콜백을 직접 spin하지 않는다. | [L208–L215](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L208-L215) |
| `sendGoal()`, `cancelLateGoal()` | goal 승인 대기와, timeout 후 뒤늦게 승인된 goal의 취소·결과 로깅을 처리한다. | [L216–L273](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L216-L273) |
| `call()` | 비동기 서비스 요청 후 5초까지 응답을 기다린다. 실패 시 pending request를 제거한다. | [L274–L281](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L274-L281) |
| `scene()`, `apply()` | 현재 Planning Scene 조회 및 diff 적용. | [L282–L287](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L282-L287), [L318–L324](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L318-L324) |
| `sphere()` | 반지름 0.0325m의 apple 충돌 물체 생성. | [L325–L333](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L325-L333) |
| `verifyJoints()` | 최종 관절 오차가 0.025rad 이하인지 최대 5초 확인한다. | [L450–L467](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L450-L467) |
| `advance()`, `fail()` | 정상 상태 전이 또는 ERROR 기록 후 실패 대기. | [L679–L688](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L679-L688) |
| 소멸자 | 종료 플래그 설정, 진행 중 계획 preempt, 작업 future 종료 대기. | [L151–L161](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L151-L161) |

## 9. 오해하기 쉬운 지점

- `operation_timeout`은 전체 Pick-and-Place의 공통 마감 시간이 아니다. MTC 실행 결과 대기에 사용되며, 서비스 5초·goal 승인 10초·접촉 대기 30초 등 별도 상수 제한도 있다.
- 시작 관절 `start_`는 설정이 비어 있으면 **준비 완료 시점**에 저장한다([L704](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L704)). 생성 직후 무조건 저장하는 것이 아니다.
- 준비 확인은 인터페이스 가용성, 컨트롤러 상태, 관측 및 장면을 검사한다. 준비 단계에서 시험 경로를 계획해 성공까지 확인하는 것은 아니다.
- 운반 중 부착 확인은 함수에 표시된 지점에서 수행한다. 모든 궤적 샘플마다 부착을 감시해 즉시 정지시키는 구현으로 해석하면 안 된다.
- 일반 작업 실패는 `FAILED_WAITING`으로 남지만, **생성자 실패 등 main의 catch로 전파된 예외**는 [L797–L799](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L797-L799)에서 노드를 종료한다.
- 이 문서는 현재 코드의 읽기 안내다. 새 런타임 시험이나 SingleThreadedExecutor 비교 성능 검증 결과를 의미하지 않는다.
