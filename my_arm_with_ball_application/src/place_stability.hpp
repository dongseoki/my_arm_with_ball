#ifndef MY_ARM_WITH_BALL_APPLICATION_PLACE_STABILITY_HPP_
#define MY_ARM_WITH_BALL_APPLICATION_PLACE_STABILITY_HPP_

#include <array>
#include <cmath>
#include <optional>

namespace pick_place
{
// Bounds are world-frame limits for the ball CENTER, already inset by its radius.
class PlaceStability
{
public:
    /**
     * @brief 공이 바구니 안에서 안정됐는지 판정할 위치·속도·시간 기준을 설정한다.
     * @param bounds world 좌표계 기준 공 중심의 허용 범위(m).
     *               순서는 {xmin, xmax, ymin, ymax, zmin, zmax}이며 경계값을 포함한다.
     *               공 전체가 바구니 안에 있도록 공 반지름을 고려해 정한 범위를 전달한다.
     * @param speed 안정 상태로 인정할 최대 관측 간 이동 속도(m/s).
     * @param duration 범위 안에서 속도 조건을 연속으로 만족해야 하는 최소 시간(초).
     * @note 입력값의 유효성은 검사하지 않으므로 호출자가 올바른 범위와 기준값을 제공해야 한다.
     */
    PlaceStability(std::array<double, 6> bounds, double speed, double duration)
    : bounds_(bounds), speed_(speed), duration_(duration) {}

    /**
     * @brief 공의 위치 관측을 반영하고 바구니 안에서 충분히 오래 안정됐는지 판정한다.
     * @param position world 좌표계 기준 공 중심의 {x, y, z} 위치(m).
     * @param stamp 현재 관측의 시각(초). 이전 관측과 같은 시간 기준을 사용해야 한다.
     * @return 공 중심이 bounds_ 안에 있고, 관측 간 계산 속도가 speed_ 이하인 상태가
     *         duration_ 이상 지속됐으면 true, 그렇지 않으면 false.
     * @details 위치 차이의 길이를 관측 시간 차이로 나누어 속도를 계산한다.
     *          첫 관측에서는 속도를 계산할 수 없어 false를 반환하며, 이후 조건을 만족한
     *          첫 관측부터 안정 시간을 누적한다. 범위 이탈, 속도 초과 또는 시간이
     *          증가하지 않은 경우 안정 시간 누적을 초기화한다.
     *          위치나 시각이 유한하지 않으면 reset()으로 관측 이력까지 초기화한다.
     * @note 연속 입력된 관측으로 판정하며, 관측 수신 간격이 너무 긴지는 검사하지 않는다.
     */
    bool update(const std::array<double, 3> & position, double stamp)
    {
        if (!std::isfinite(stamp) || !std::isfinite(position[0]) ||
            !std::isfinite(position[1]) || !std::isfinite(position[2]))
        {
            reset();
            return false;
        }
        const bool inside = position[0] >= bounds_[0] && position[0] <= bounds_[1] &&
            position[1] >= bounds_[2] && position[1] <= bounds_[3] &&
            position[2] >= bounds_[4] && position[2] <= bounds_[5];
        bool slow = false;
        if (previous_stamp_ && stamp > *previous_stamp_) {
            double distance_squared = 0.0;
            for (size_t i = 0; i < 3; ++i) {
                distance_squared += std::pow(position[i] - previous_position_[i], 2);
            }
            last_speed_ = std::sqrt(distance_squared) / (stamp - *previous_stamp_);
            slow = last_speed_ <= speed_;
        }
        if (!inside || !slow) {
            stable_since_.reset();
        } else if (!stable_since_) {
            stable_since_ = stamp;
        }
        previous_stamp_ = stamp;
        previous_position_ = position;
        return stable_since_ && stamp - *stable_since_ >= duration_;
    }

    void reset()
    {
        previous_stamp_.reset();
        stable_since_.reset();
        last_speed_ = 0.0;
    }
    double last_speed() const {return last_speed_;}

private:
    std::array<double, 6> bounds_;
    double speed_, duration_, last_speed_{0.0};
    std::array<double, 3> previous_position_{};
    std::optional<double> previous_stamp_, stable_since_;
};
}  // namespace pick_place
#endif  // MY_ARM_WITH_BALL_APPLICATION_PLACE_STABILITY_HPP_
