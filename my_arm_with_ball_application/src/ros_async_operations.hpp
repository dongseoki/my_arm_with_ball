#ifndef MY_ARM_WITH_BALL_APPLICATION_ROS_ASYNC_OPERATIONS_HPP_
#define MY_ARM_WITH_BALL_APPLICATION_ROS_ASYNC_OPERATIONS_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace pick_place
{
namespace ros_async
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
/**
 * @brief goal 응답 대기가 실패한 뒤 확인된 goal에 비동기 취소를 요청한다.
 * @tparam ClientPtr Action 클라이언트 공유 포인터 자료형.
 * @tparam GoalHandlePtr goal handle 공유 포인터 자료형.
 * @param client 취소와 최종 결과 조회에 사용할 Action 클라이언트.
 * @param handle 취소할 goal handle.
 * @param label 로그에 표시할 작업 설명.
 * @details 취소 요청 수락 여부와 최종 결과를 콜백으로 기록한다. 취소 완료까지 기다리지는 않는다.
 */
template <class ClientPtr, class GoalHandlePtr>
void cancelLateGoal(const ClientPtr &client, const GoalHandlePtr &handle,
                    const std::string &label, const rclcpp::Logger &logger)
{
    RCLCPP_ERROR(logger, "%s goal response arrived after timeout; cancelling the late goal", label.c_str());
    try
    {
        client->async_cancel_goal(handle, [logger, label](const auto &response)
                                  {
            if (response->goals_canceling.empty()) {
                RCLCPP_ERROR(logger, "%s late-goal cancel request was not accepted", label.c_str());
            } });
        client->async_get_result(handle, [logger, label](const auto &result)
                                 {
            if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_WARN(logger, "%s late goal cancellation confirmed", label.c_str());
            } else {
                RCLCPP_ERROR(logger, "%s late goal terminal result was not cancellation (code=%d)",
                    label.c_str(), static_cast<int>(result.code));
            } });
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(logger, "%s late goal cancellation could not be confirmed: %s", label.c_str(), ex.what());
    }
}

/**
 * @brief Future의 완료를 제한 시간 동안 기다린 뒤 결과를 가져온다.
 * @tparam Future wait_for()와 get()을 제공하는 비동기 결과 자료형.
 * @param future 결과를 기다릴 Future.
 * @param timeout 실제 경과 시간 기준 대기 제한(초).
 * @param label 시간 초과·종료 예외에 표시할 작업 설명.
 * @return future.get()으로 얻은 결과.
 * @throws std::runtime_error 대기 중 종료가 요청되거나 ROS가 종료되거나 제한 시간을 초과한 경우.
 * @note Future 자체에 저장된 예외도 호출자에게 전달된다. tick()이 아닌 작업 스레드에서 사용한다.
 */
template <class Future>
auto await(std::atomic<bool> &stopping, Future &&future, double timeout, const std::string &label)
{
    const auto begin = Clock::now();
    while (future.wait_for(50ms) != std::future_status::ready)
    {
        if (stopping || !rclcpp::ok() || std::chrono::duration<double>(Clock::now() - begin).count() > timeout)
        {
            throw std::runtime_error(label + " timeout/shutdown");
        }
    }
    return future.get();
}
/**
 * @brief Action goal을 보내고 서버의 승인 또는 거부 응답을 기다린다.
 * @tparam ClientPtr Action 클라이언트 공유 포인터 자료형.
 * @tparam Goal 전송할 Action goal 자료형.
 * @param client goal을 보낼 Action 클라이언트.
 * @param goal 전송할 작업 목표.
 * @param timeout goal 응답 대기 제한(초). 작업 실행 완료 제한은 아니다.
 * @param label 응답 대기와 취소 로그에 표시할 작업 설명.
 * @return 승인된 goal handle. 거부되면 빈 포인터.
 * @details 응답 대기가 실패한 뒤 승인된 goal도 취소하도록 응답 콜백과 상태를 공유한다.
 * @note 전송·대기 중 발생한 예외는 취소 요청 처리 후 호출자에게 전달한다.
 */
template <class ClientPtr, class Goal>
auto sendGoal(std::atomic<bool> &stopping, const rclcpp::Logger &logger,
              const ClientPtr &client, const Goal &goal, double timeout, const std::string &label)
{
    using Client = typename std::decay_t<ClientPtr>::element_type;
    using GoalHandle = typename Client::GoalHandle;
    struct ResponseState
    {
        std::mutex mutex;
        bool abandoned{false};
        typename GoalHandle::SharedPtr handle;
    };
    auto state = std::make_shared<ResponseState>();
    typename Client::SendGoalOptions options;
    options.goal_response_callback = [client, state, label, logger](typename GoalHandle::SharedPtr handle)
    {
        bool cancel = false;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->handle = handle;
            cancel = state->abandoned && static_cast<bool>(handle);
        }
        if (cancel)
        {
            cancelLateGoal(client, handle, label, logger);
        }
    };
    auto future = client->async_send_goal(goal, options);
    try
    {
        return await(stopping, future, timeout, label);
    }
    catch (...)
    {
        typename GoalHandle::SharedPtr handle;
        {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->abandoned = true;
            handle = state->handle;
        }
        if (handle)
        {
            cancelLateGoal(client, handle, label, logger);
        }
        throw;
    }
}

}  // namespace ros_async
}  // namespace pick_place
#endif  // MY_ARM_WITH_BALL_APPLICATION_ROS_ASYNC_OPERATIONS_HPP_
