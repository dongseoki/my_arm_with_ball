# `/grasp/attach`와 `/grasp/detach` 서비스 설명

## 한눈에 보기

`/grasp/attach`와 `/grasp/detach`는 이 프로젝트의 Gazebo 플러그인이 제공하는 ROS 2 서비스다. Gazebo 물리 시뮬레이션에서 공을 그리퍼 손 링크에 고정하거나 고정을 해제한다. MoveIt 자체의 표준 서비스가 아니며, MoveIt의 Planning Scene에서 공을 부착/분리하는 작업과는 별개다.

| 이름 | 타입 | 역할 |
| --- | --- | --- |
| `/grasp/attach` | `std_srvs/srv/Trigger` | 접촉·그리퍼 조건을 확인한 뒤 공과 `rg2_hand` 사이에 Gazebo 고정 연결을 만든다. |
| `/grasp/detach` | `std_srvs/srv/Trigger` | 그 연결을 제거해 공이 중력과 물리 법칙에 따라 움직이도록 한다. |

`Trigger` 요청에는 별도 입력값이 없다. 응답의 `success`와 `message`가 작업 결과를 전달한다. 이 ROS 인터페이스 자체는 범용 Trigger지만, 실제 동작과 조건은 프로젝트의 플러그인 코드가 정한다.

## 어디에 정의되어 있나

1. **서비스 서버 및 Gazebo 동작:** `my_arm_with_ball_gazebo/src/GraspSystem.cc`의 `Configure()`에서 두 서비스를 `/grasp/attach`, `/grasp/detach` 이름으로 생성한다. 각 요청은 내부 `Request(Command::Attach/Detach, ...)`를 거쳐 Gazebo 시뮬레이션 업데이트에서 처리된다.
2. **플러그인 로딩:** `my_arm_with_ball_gazebo/worlds/apple_pick_place.sdf`가 `libGraspSystem.so` 플러그인을 world에 등록한다. 라이브러리 빌드·설치는 `my_arm_with_ball_gazebo/CMakeLists.txt`의 `GraspSystem` 타깃에 정의돼 있다.
3. **테스트 클라이언트:** `my_arm_with_ball_application/scripts/grasp_test.py`에서 `Trigger` 클라이언트를 만들고 실제 잡기/놓기 흐름에서 호출한다.

따라서 서비스 이름은 Gazebo 플러그인 코드에서 정해지고, world 파일을 통해 시뮬레이션에 들어온다. ROS 2에서 확인할 때는 실행 중 `ros2 service type /grasp/attach` 또는 `ros2 service type /grasp/detach`를 사용하면 타입을 볼 수 있다.

## `/grasp/attach`가 하는 일

공을 집는 순간, 플러그인은 임의로 공을 붙이지 않고 다음 조건을 검사한다.

- 로봇 손과 공의 Gazebo 엔티티가 발견되어 있을 것.
- 직전 3초 안에 손가락-공 접촉이 관측되었을 것.
- 양쪽 손가락 관절 위치의 합이 5°보다 클 것(완전히 닫힌 빈 그리퍼를 잘못 집기로 판정하지 않기 위한 조건).

조건을 만족하면 Gazebo Entity Component Manager에 `DetachableJoint`를 만들어 `rg2_hand`와 공의 `ball` 링크를 `fixed` 연결로 묶는다. 이후 손 링크가 움직이면 공도 물리적으로 함께 움직인다. 조건 미충족, 아직 준비되지 않은 엔티티 등은 실패 응답을 돌려준다. 이미 연결된 상태는 성공으로 처리된다.

3초 접촉 기록은 닫힘 궤적이 접촉 직후 취소되는 사이 접촉 상태가 잠깐 사라지는 경우를 고려한 짧은 확인 창이다. 그래도 요청 시점에 손가락 관절 개방 조건을 다시 확인한다.

## `/grasp/detach`가 하는 일

플러그인이 보관 중인 `DetachableJoint` 엔티티를 제거한다. 이때 공을 바구니로 순간 이동시키거나 Planning Scene을 바꾸지 않는다. 연결이 풀린 뒤에는 Gazebo 물리 엔진이 공의 위치와 낙하를 계산한다. 현재 구현은 연결이 없더라도 detach 요청을 성공으로 응답한다.

## `grasp_test.py`에서의 순서

테스트 노드는 먼저 Gazebo 접촉 상태(`/grasp/contact`)를 기다리고, 접촉 시 그리퍼 닫기 동작을 멈춘 다음 `/grasp/attach`를 호출한다. 들어올려 공이 손을 따라오는지 확인한 뒤 `/grasp/detach`를 호출하고, 공이 분리 상태로 떨어지는지 확인한다.

이 테스트는 Gazebo와 MoveIt 두 상태를 따로 맞춘다.

| 시스템 | 부착/분리 방식 | 의미 |
| --- | --- | --- |
| Gazebo 물리 | `/grasp/attach`, `/grasp/detach` | 실제 시뮬레이션에서 공과 손 사이의 물리 연결을 생성/제거한다. |
| MoveIt Planning Scene | `grasp_scene.py`의 `ApplyPlanningScene` 요청 | 충돌 계획을 위해 공을 월드 오브젝트에서 로봇 링크에 부착된 collision object로 바꾸거나, 월드 오브젝트로 복원한다. |

`grasp_test.py`는 Gazebo attach 이후 `attach_ball()`로 MoveIt 장면도 갱신하며, Gazebo detach 이후 `detach_ball()`로 MoveIt 장면을 복원한다. Gazebo 서비스만 호출하면 MoveIt 장면은 자동으로 바뀌지 않고, 반대로 MoveIt Planning Scene만 갱신해도 Gazebo 공이 물리적으로 손에 붙지는 않는다. 테스트가 두 전환을 별도로 수행하는 이유다.

## 혼동하면 안 되는 점

- `/grasp/attach`·`/grasp/detach`는 이 저장소의 Gazebo 통합용 사용자 정의 인터페이스다. MoveIt의 `attach object`라는 개념과 이름은 같지만 같은 기능 층이 아니다.
- `/grasp/attached`는 서비스가 아니라 플러그인이 발행하는 `std_msgs/msg/Bool` 토픽이다. 현재 Gazebo 고정 연결 여부를 관찰하는 상태 신호다. `/grasp/contact`도 접촉 여부를 알려주는 Bool 토픽이다.
- MoveIt 장면 전환은 `my_arm_with_ball_application/scripts/grasp_scene.py`에 구현돼 있으며, Gazebo 서비스 서버가 이를 호출하지 않는다. 테스트 클라이언트가 순서를 조정한다.
- 이 연결은 단순한 테스트용 고정 조인트 모델이다. 힘/마찰만으로 안정적으로 집는 물리적 그립 동작 그 자체를 대신하는 구현은 아니다.

## 호출 예시

서비스 타입과 서버 존재 여부를 확인할 수 있다.

```bash
ros2 service type /grasp/attach
ros2 service call /grasp/attach std_srvs/srv/Trigger {}
ros2 service call /grasp/detach std_srvs/srv/Trigger {}
ros2 topic echo /grasp/attached
```

attach 호출은 접촉 및 개방각 조건을 충족하지 않으면 실패해야 한다. 예시 명령은 실행 중인 시뮬레이션 상태를 바꾸므로, 자동 테스트에서는 `grasp_test.py`처럼 전후 상태를 확인하며 사용해야 한다.
