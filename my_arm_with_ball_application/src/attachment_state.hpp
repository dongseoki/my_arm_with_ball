#ifndef MY_ARM_WITH_BALL_APPLICATION_ATTACHMENT_STATE_HPP_
#define MY_ARM_WITH_BALL_APPLICATION_ATTACHMENT_STATE_HPP_
namespace pick_place
{
/// @brief 각 시스템에서 기대하는 부착 상태. Gazebo와 MoveIt 검사는 각각 수행한다.
enum class AttachmentState { Attached, Detached };
}
#endif  // MY_ARM_WITH_BALL_APPLICATION_ATTACHMENT_STATE_HPP_
