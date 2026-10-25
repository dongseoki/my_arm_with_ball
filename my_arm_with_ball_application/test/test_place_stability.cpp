#include <array>
#include <limits>
#include <stdexcept>

#include "../src/place_stability.hpp"

int main()
{
    // 1. 기대한 조건이 거짓이면 예외를 발생시켜 테스트를 실패시키는 검사 함수를 정의한다.
    auto check = [](bool condition) {
        if (!condition) {throw std::runtime_error("place stability regression");}
    };
    // 2. 공 중심의 허용 범위(m), 안정 속도 상한(0.02 m/s), 지속 시간(1초)을 준비한다.
    const std::array<double, 6> bounds{0.71, 0.79, 0.244, 0.256, 0.033, 0.11};
    pick_place::PlaceStability stability(bounds, 0.02, 1.0);
    const std::array<double, 3> inside{0.75, 0.25, 0.04};
    const std::array<double, 3> outside{0.80, 0.25, 0.04};

    // 3. 첫 관측만으로는 성공하지 않고, 속도 조건을 만족한 0.1초부터 1초가 지나야 성공하는지 확인한다.
    check(!stability.update(inside, 0.0));
    check(!stability.update(inside, 0.1));
    check(!stability.update(inside, 1.0));
    check(stability.update(inside, 1.1));

    // 4. 바구니 밖으로 나가면 안정 시간이 초기화되고, 다시 들어온 뒤 1초간 안정돼야 성공하는지 확인한다.
    check(!stability.update(outside, 1.2));
    check(!stability.update(inside, 1.3));
    check(!stability.update(inside, 1.4));
    check(stability.update(inside, 2.4));

    // 5. 같은 관측 시각이 반복되면 안정 판정이 해제되고, 시간이 진행된 뒤 다시 1초를 누적하는지 확인한다.
    check(!stability.update(inside, 2.4));  // 시뮬레이션 시간이 정지한 상황을 재현한다.
    check(!stability.update(inside, 2.5));
    check(stability.update(inside, 3.5));

    // 6. 0.1초 동안 0.01 m 이동하면 속도 상한을 초과하므로 성공 판정을 해제하는지 확인한다.
    check(!stability.update({0.76, 0.25, 0.04}, 3.6));  // 계산 속도는 약 0.1 m/s이다.
    check(stability.last_speed() > 0.02);
    // 7. 유효하지 않은 시각(NaN)은 관측 이력을 초기화하고, 이후 정상 관측으로 다시 성공할 수 있는지 확인한다.
    check(!stability.update(inside, std::numeric_limits<double>::quiet_NaN()));
    check(!stability.update(inside, 4.0));
    check(!stability.update(inside, 4.1));
    check(stability.update(inside, 5.1));
}
