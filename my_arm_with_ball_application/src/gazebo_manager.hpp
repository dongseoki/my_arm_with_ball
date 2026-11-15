#ifndef MY_ARM_WITH_BALL_APPLICATION_GAZEBO_MANAGER_HPP_
#define MY_ARM_WITH_BALL_APPLICATION_GAZEBO_MANAGER_HPP_
#include "attachment_state.hpp"
#include <atomic>
#include <functional>
#include <optional>
#include <std_srvs/srv/trigger.hpp>
namespace pick_place
{
/// @brief Gazebo 물리 부착·분리 및 최신 관측 확인만 담당한다. 접촉 닫기는 노드에 남긴다.
class GazeboManager
{
public:
    using RequestAttachment = std::function<std_srvs::srv::Trigger::Response::SharedPtr(bool)>;
    /// @brief 잠금 아래 읽은 최신 부착 관측. 누락·오래된 관측은 nullopt이다.
    using Observation = std::function<std::optional<bool>()>;
    GazeboManager(std::atomic<bool> &stopping, RequestAttachment request_attachment, Observation observation);
    void requireGazeboAttachment(AttachmentState expected);
    void setGazeboAttachment(AttachmentState requested);
private:
    std::atomic<bool> &stopping_;
    RequestAttachment request_attachment_;
    Observation observation_;
};
}
#endif  // MY_ARM_WITH_BALL_APPLICATION_GAZEBO_MANAGER_HPP_
