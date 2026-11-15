#include "gazebo_manager.hpp"
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>
namespace pick_place
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
namespace
{
double secondsSince(Clock::time_point time)
{
    return std::chrono::duration<double>(Clock::now() - time).count();
}
}
GazeboManager::GazeboManager(std::atomic<bool> &stopping,
    RequestAttachment request_attachment, Observation observation)
    : stopping_(stopping), request_attachment_(std::move(request_attachment)),
      observation_(std::move(observation))
{
}

/**
 * @brief Gazebo의 최신 공 부착 상태가 기대값과 일치하는지 검사한다.
 * @param expected 기대하는 부착 상태. Attached는 부착, Detached는 분리 상태이다.
 * @throws std::runtime_error 부착 관측이 미수신·오래된 값이거나 기대 상태와 다른 경우.
 * @note 상태를 검사할 뿐 실제 부착·분리나 Planning Scene 변경은 수행하지 않는다.
 */
void GazeboManager::requireGazeboAttachment(AttachmentState expected)
{
    const bool value = expected == AttachmentState::Attached;
    if (observation_().value_or(!value) != value)
    {
        throw std::runtime_error("Gazebo attachment state mismatch/stale");
    }
}

/**
 * @brief Gazebo에 공 부착 또는 분리를 요청하고 실제 관측으로 완료를 확인한다.
 * @param requested Attached이면 부착 서비스, Detached이면 분리 서비스를 호출한다.
 * @details 서비스 성공 후 최신 부착 관측이 요청 상태와 일치할 때까지 최대 3초 기다린다.
 *          MoveIt Planning Scene 동기화는 수행하지 않는다.
 * @throws std::runtime_error 서비스 호출·요청이 실패하거나 확인 대기 중 종료 또는 시간 초과가 발생한 경우.
 */
void GazeboManager::setGazeboAttachment(AttachmentState requested)
{
    const bool attach = requested == AttachmentState::Attached;
    auto response = request_attachment_(attach);
    if (!response->success)
    {
        throw std::runtime_error(response->message);
    }
    const auto begin = Clock::now();
    while (true)
    {
        if (observation_().value_or(!attach) == attach)
        {
            return;
        }
        if (stopping_ || secondsSince(begin) > 3.)
        {
            throw std::runtime_error("Gazebo attachment confirmation timeout");
        }
        std::this_thread::sleep_for(20ms);
    }
}
}  // namespace pick_place
