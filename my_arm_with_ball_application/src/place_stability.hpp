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
    PlaceStability(std::array<double, 6> bounds, double speed, double duration)
    : bounds_(bounds), speed_(speed), duration_(duration) {}

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
