#include <array>
#include <limits>
#include <stdexcept>

#include "../src/place_stability.hpp"

int main()
{
    auto check = [](bool condition) {
        if (!condition) {throw std::runtime_error("place stability regression");}
    };
    const std::array<double, 6> bounds{0.71, 0.79, 0.244, 0.256, 0.033, 0.11};
    pick_place::PlaceStability stability(bounds, 0.02, 1.0);
    const std::array<double, 3> inside{0.75, 0.25, 0.04};
    const std::array<double, 3> outside{0.80, 0.25, 0.04};

    check(!stability.update(inside, 0.0));
    check(!stability.update(inside, 0.1));
    check(!stability.update(inside, 1.0));
    check(stability.update(inside, 1.1));

    check(!stability.update(outside, 1.2));
    check(!stability.update(inside, 1.3));
    check(!stability.update(inside, 1.4));
    check(stability.update(inside, 2.4));

    check(!stability.update(inside, 2.4));  // Paused simulation time.
    check(!stability.update(inside, 2.5));
    check(stability.update(inside, 3.5));

    check(!stability.update({0.76, 0.25, 0.04}, 3.6));  // Too fast.
    check(stability.last_speed() > 0.02);
    check(!stability.update(inside, std::numeric_limits<double>::quiet_NaN()));
    check(!stability.update(inside, 4.0));
    check(!stability.update(inside, 4.1));
    check(stability.update(inside, 5.1));
}
