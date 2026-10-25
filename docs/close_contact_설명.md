# `closeContact()` 라인별 설명

이 문서는 [`pick_and_place_test.cpp`의 `closeContact()` 함수](../my_arm_with_ball_application/src/pick_and_place_test.cpp#L892-L991)를 코드 순서대로 설명한다. 아래 코드는 현재 구현을 옮긴 것이며, 각 코드 줄의 오른쪽에 설명을 붙였다.

## 1. 함수 코드와 줄별 설명

```cpp
    void closeContact()                                                        // 그리퍼 접촉·닫기·Gazebo 부착 요청을 처리하는 함수
    {                                                                           // 함수 본문 시작
        MoveGroup::Goal goal;                                                   // MoveGroup Action에 보낼 목표 메시지
        auto &request = goal.request;                                           // 목표 메시지 안의 모션 계획 요청을 참조
        request.group_name = "gripper";                                        // 제어할 MoveIt 관절 그룹을 그리퍼로 지정
        request.pipeline_id = "ompl";                                          // 계획 파이프라인으로 OMPL을 지정
        request.planner_id = "RRTConnectkConfigDefault";                      // OMPL 내부 경로 계획기를 RRTConnect로 지정
        request.allowed_planning_time = planning_time_;                        // 계획에 허용할 시간을 파라미터 값으로 지정
        request.num_planning_attempts = 5;                                      // 계획 시도 횟수를 5회로 지정
        request.max_velocity_scaling_factor = velocity_;                       // 최대 속도 비율을 설정
        request.max_acceleration_scaling_factor = acceleration_;               // 최대 가속도 비율을 설정
        request.start_state.is_diff = true;                                     // 현재 상태를 바탕으로 요청 상태를 부분 갱신
        moveit_msgs::msg::Constraints constraints;                              // 그리퍼 목표 관절 제약을 담을 메시지
        for (const auto &name : FINGERS)                                        // 두 손가락 관절을 각각 순회
        {                                                                       // 반복문 본문 시작
            moveit_msgs::msg::JointConstraint joint;                            // 현재 손가락 하나의 목표 관절 제약
            joint.joint_name = name;                                            // 제약을 적용할 관절 이름
            joint.position = .2;                                                // 해당 관절의 목표 각도를 0.2 rad로 설정
            joint.tolerance_above = .005;                                       // 목표보다 위쪽으로 허용할 오차(0.005 rad)
            joint.tolerance_below = .005;                                       // 목표보다 아래쪽으로 허용할 오차(0.005 rad)
            joint.weight = 1.;                                                  // 이 관절 제약의 상대 가중치
            constraints.joint_constraints.push_back(joint);                    // 현재 관절 제약을 전체 제약 목록에 추가
        }                                                                       // 두 손가락 관절 설정 완료
        request.goal_constraints.push_back(constraints);                        // 구성한 두 손가락 목표를 계획 요청에 넣음
        goal.planning_options.planning_scene_diff.is_diff = true;               // 계획용 장면 변경분을 사용하도록 표시
        goal.planning_options.planning_scene_diff.robot_state.is_diff = true;   // 계획용 로봇 상태도 변경분으로 표시
        auto handle = sendGoal(move_, goal, 10., "close acceptance");          // MoveGroup에 goal 전송, 승인 응답을 최대 10초 기다림
        if (!handle)                                                            // Action 서버가 goal을 승인하지 않았는지 검사
        {                                                                       // 거부 처리 시작
            throw std::runtime_error("Gripper close rejected");                // 거부를 실패 예외로 전달
        }                                                                       // 거부 처리 종료
        auto result_future = move_->async_get_result(handle);                   // goal의 최종 실행 결과를 비동기로 요청
        try                                                                     // 실패 시에도 진행 중 goal을 정리하기 위한 예외 처리 시작
        {                                                                       // 본 처리 시작
            const auto begin = Clock::now();                                    // 접촉 대기 제한시간 측정 시작
            bool contact = false;                                               // 접촉을 아직 확인하지 못한 상태로 초기화
            while (true)                                                        // 접촉 또는 Action 종료까지 반복 확인
            {                                                                   // 대기 반복 시작
                {                                                               // 공유 접촉 상태를 잠깐 보호하는 영역
                    std::lock_guard<std::mutex> lock(mutex_);                   // 구독 콜백과 공유 상태 접근을 동기화
                    contact = fresh(contact_received_) && contact_.value_or(false); // 접촉값이 최신이고 true인지 검사
                }                                                               // 잠금 해제
                if (contact || result_future.wait_for(20ms) == std::future_status::ready) // 접촉했거나 Action이 끝났는지 확인
                {                                                               // 반복을 끝내야 하는 경우
                    break;                                                      // 대기 반복 종료
                }                                                               // 반복 종료 조건 검사 끝
                if (stopping_ || secondsSince(begin) > 30.)                    // 종료 요청 또는 30초 초과 여부 확인
                {                                                               // 제한시간 초과 처리 시작
                    throw std::runtime_error("Finger contact timeout");        // 접촉 대기 실패를 예외로 전달
                }                                                               // 제한시간 초과 처리 끝
            }                                                                   // 접촉 대기 반복 끝
            if (!contact)                                                       // Action이 접촉 없이 끝난 경우 확인
            {                                                                   // 접촉 실패 처리 시작
                throw std::runtime_error("Gripper finished closing without contact"); // 접촉 없는 닫기를 실패로 처리
            }                                                                   // 접촉 실패 처리 끝
            if (result_future.wait_for(0ms) != std::future_status::ready)       // 접촉 시점에도 닫기 Action이 진행 중인지 검사
            {                                                                   // 아직 진행 중일 때 취소
                const auto cancel = await(move_->async_cancel_goal(handle), 5., "close cancellation"); // 취소 요청 응답을 최대 5초 대기
                if (cancel->goals_canceling.empty())                            // 취소 요청이 수락되지 않았는지 확인
                {                                                               // 취소 거부 처리 시작
                    throw std::runtime_error("Close cancellation not accepted"); // 취소 거부를 실패 예외로 전달
                }                                                               // 취소 거부 처리 끝
            }                                                                   // 진행 중 Action 취소 처리 끝
            const auto result = await(result_future, 10., "close terminal result"); // Action의 최종 결과를 최대 10초 대기
            if (result.code != rclcpp_action::ResultCode::SUCCEEDED && result.code != rclcpp_action::ResultCode::CANCELED) // 허용된 종료 결과인지 확인
            {                                                                   // 실패 종료 처리 시작
                throw std::runtime_error("Close did not terminate successfully/canceled"); // 예상하지 못한 결과를 실패로 처리
            }                                                                   // 실패 종료 처리 끝
            const auto q = measured();                                          // 최신 팔·그리퍼 관절값을 가져옴
            if (q.at(FINGERS[0]) < -.002 || q.at(FINGERS[1]) < -.002 ||         // 손가락 관절값의 비정상 음수 여부 확인
                q.at(FINGERS[0]) + q.at(FINGERS[1]) <= 5. * M_PI / 180.)        // 두 손가락 합산 개방량이 5도 이하인지 확인
            {                                                                   // 관절 상태가 허용 범위를 벗어난 경우
                throw std::runtime_error("Contact close opening <= 5 degrees"); // 손가락 상태 검증 실패를 예외로 전달
            }                                                                   // 관절 상태 검증 끝
            // Gazebo requires recent contact: request attachment immediately after close termination. // 접촉 정보가 유효할 때 바로 부착 요청
            trigger(true);                                                      // Gazebo 부착 서비스를 호출하고 부착 상태 확인
        }                                                                       // 정상 처리 블록 끝
        catch (...)                                                             // 위 처리에서 예외가 발생한 경우
        {                                                                       // 정리 처리 시작
            if (result_future.wait_for(0ms) != std::future_status::ready)       // Action이 아직 끝나지 않았는지 검사
            {                                                                   // 진행 중이면 취소 시도
                try                                                             // 취소·종료 확인 중 추가 예외를 별도 처리
                {                                                               // 정리 시도 시작
                    await(move_->async_cancel_goal(handle), 5., "failed close cancel"); // 진행 중인 닫기 goal 취소를 최대 5초 대기
                    await(result_future, 10., "failed close terminal");        // 취소 후 최종 결과를 최대 10초 대기
                }                                                               // 정리 시도 끝
                catch (const std::exception &cleanup)                           // 정리 절차 자체가 실패한 경우
                {                                                               // 정리 실패 로깅 시작
                    RCLCPP_ERROR(get_logger(), "Close goal terminal state UNCONFIRMED after cancellation: %s", cleanup.what()); // 최종 상태 미확인을 오류 로그로 기록
                }                                                               // 정리 실패 로깅 끝
            }                                                                   // 필요 시 취소·종료 확인 끝
            throw;                                                              // 원래 발생한 예외를 상위 작업 상태 처리로 다시 전달
        }                                                                       // 예외 처리 끝
    }                                                                           // closeContact 함수 끝
```

## 2. 실행 흐름 요약

```text
MoveGroup에 그리퍼 닫기 목표 전송
    ↓
접촉 감지와 Action 종료를 함께 확인
    ├─ Action이 먼저 끝났는데 접촉 없음 → 실패
    ├─ 30초 안에 접촉 없음 → 실패
    └─ 접촉 감지
          ↓
       닫기 Action이 아직 진행 중이면 취소
          ↓
       최종 Action 결과 확인
          ↓
       실제 손가락 관절값과 5도 기준 확인
          ↓
       trigger(true): Gazebo 공 부착 요청 및 상태 확인
```

### 읽을 때 주의할 점

- 이 함수는 MTC의 `Task`를 만들지 않고 `MoveGroup` Action으로 그리퍼 목표를 요청한다.
- `.2`와 `.005`는 관절 위치·허용 오차 값이며, 여기서는 라디안 단위다.
- 접촉 여부는 `contact_` 값만 보는 것이 아니라 `fresh(contact_received_)`로 관측 최신성도 확인한다.
- 접촉 후 닫기 Action이 이미 끝났다면 취소를 보내지 않고 최종 결과를 확인한다. 진행 중이면 취소 응답을 확인한 뒤 최종 결과를 기다린다.
- 이 함수의 성공은 `trigger(true)`가 반환해야 확정된다. 해당 함수는 Gazebo 부착 서비스 성공과 `/grasp/attached` 관측을 확인한다.
- 함수 내부 예외는 호출자인 `tick()`의 작업 결과 처리로 전달되어 실패 상태 기록에 사용된다.
