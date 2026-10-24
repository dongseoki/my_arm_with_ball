# pick_and_place_test.cpp 실장비 연동 시 변경 포인트

## 개요

실제 로봇으로 옮길 때는 Gazebo의 가상 부착 코드를 덜어내고, 접촉·물체 위치·파지 상태 입력을 실장비 데이터로 대체해야 한다. `syncScene()`이나 MTC까지 제거하는 것은 아니다.

이 문서는 현재 코드에 대한 검토 결과이며, 실장비 연동 구현이나 구동 검증을 완료했다는 의미는 아니다.

검토 대상: [`pick_and_place_test.cpp`](../my_arm_with_ball_application/src/pick_and_place_test.cpp)

## 1. 제거 대상: Gazebo의 가상 부착·분리

| 위치 | 덜어낼 내용 |
|---|---|
| [156~157행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L156-L157) | `/grasp/attach`, `/grasp/detach` 서비스 클라이언트 |
| [130~134행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L130-L134) | Gazebo 부착 상태인 `/grasp/attached` 구독 |
| [638~670행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L638-L670) | `requireAttachment()`와 `trigger()`의 Gazebo 의존 구현 |
| [973~975행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L973-L975) | `closeContact()` 마지막의 `trigger(true)` |
| [1077~1085행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L1077-L1085) | `release()` 내부의 `requireAttachment(true)`, `trigger(false)` |
| `transport()` 및 상태 분기 | 여러 곳의 `requireAttachment(...)` 호출 |

이 코드는 Gazebo에서 공과 손을 가상으로 결합하거나 그 상태를 확인한다. 실제 장비에서는 그리퍼의 물리적인 파지와 센서 기반 확인이 그 역할을 맡아야 한다.

따라서 Gazebo 부착 검사를 제거하더라도 **실제 파지 여부를 확인하는 기능 자체를 없애서는 안 된다.** 관련 클라이언트·구독 멤버와 `readiness()`의 대기 조건도 함께 정리하거나 교체해야 한다.

## 2. 단순 삭제가 아니라 대체할 부분

| 위치 | 실제 장비 전환 시 처리 |
|---|---|
| [125~129행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L125-L129)의 `/grasp/contact` | 실제 그리퍼의 접촉·힘·전류·파지 상태 등 사용 가능한 피드백으로 대체 |
| [135~152행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L135-L152)의 `/grasp/ball_pose`, `ball()` | 카메라·위치 추정 또는 사전에 알고 있는 물체 위치로 대체. `world` 좌표계와 데이터 최신성 조건도 검토 |
| [closeContact()](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L894-L994) | 접촉 대기와 취소 방식을 실제 그리퍼 드라이버에 맞게 변경. 두 손가락 관절 모델, 관절 목표 `.2`, 개방량 판정도 재검토 |
| [readiness()](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L377-L435) | Gazebo 토픽·서비스 대기 조건을 실제 드라이버·센서 준비 상태로 교체 |
| [1044~1057행](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L1044-L1057)의 손–공 위치 보정 | Gazebo 공 위치 대신 실제 측정값이나 파지 오프셋 사용 |
| [VERIFY_PLACE 처리](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L1208-L1235) | 실제 물체 위치를 관측할 수 있으면 유지·대체. 관측 수단 없이 제거하면 놓기 성공을 검증하는 기능도 사라짐 |

위 `/grasp/*` 인터페이스는 [Gazebo의 `GraspSystem.cc`](../my_arm_with_ball_gazebo/src/GraspSystem.cc#L56-L64)에서 제공한다. 따라서 관련 대기 조건을 그대로 두면 실장비에서는 준비 단계나 접촉 대기에서 진행하지 못할 수 있다.

`closeContact()` 전체가 Gazebo 전용 코드는 아니다. 닫기 명령, 결과 확인, 타임아웃·취소 처리의 목적은 실장비에서도 필요하지만, 실제 그리퍼가 현재 명령·취소 방식을 지원하는지는 확인해야 한다.

## 3. 유지할 부분

### Planning Scene 관리

- [`syncScene()`](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L563-L637), [`apply()`](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L438-L448), [`verifyScene()`](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L498-L562)는 유지 대상이다.
- 실제 로봇에서도 MoveIt이 공을 손에 붙은 충돌 물체로 인식하고, 놓은 뒤에는 world 물체로 인식해야 한다.
- `syncScene()`의 공 위치 입력은 실제 측정·추정 방식으로 교체해야 한다.
- `release()`의 `trigger(false)`는 Gazebo 분리 요청이지만, `syncScene(false)`는 MoveIt 장면 갱신이므로 구분해야 한다.
- `ATTACH`와 `REFRESH_SCENE` 상태에서도 Gazebo 확인 부분과 Planning Scene 갱신 부분을 구분한다.

### MTC 경로 계획·실행

- `approach()`, 들어 올리기·운반·그리퍼 열기·후퇴·복귀 Task의 기본 구조는 유지 대상이다.
- 시작·접근 관절값, 목표 위치, 바구니 경계 등 현재 환경에 맞춘 값은 실제 작업 환경에 맞게 보정해야 한다.

### 관절 상태와 실행 검증

- `/joint_states` 구독, 실행 결과·관절값 검증, 타임아웃·실패 처리는 실장비에서도 필요하다.
- 액션 이름·관절 이름·컨트롤러 이름은 실제 드라이버 구성과 맞춰야 한다.
- 손과 공 사이의 충돌 허용 설정도 실제 그리퍼 형상과 접촉 정책에 맞춰 검토해야 한다.

## 결론 및 한계

- **덜어낼 것:** Gazebo에서 공을 가상으로 붙이고 떼는 코드.
- **대체할 것:** 시뮬레이터가 제공하던 접촉·물체 위치·파지 상태.
- **유지할 것:** MoveIt/MTC 계획·실행·Planning Scene 관리와 실행 검증 구조.

실제 로봇·그리퍼 드라이버와 센서 구성이 정해지지 않았으므로, 어떤 인터페이스로 대체할지까지는 현재 코드만으로 확정할 수 없다.

목표 좌표·충돌 환경·속도 및 안전 조건도 별도로 검증해야 한다. 위 코드 제거만으로 실장비 구동 준비가 끝나는 것은 아니다.
