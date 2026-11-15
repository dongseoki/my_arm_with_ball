#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pick_and_place_test_node.hpp"

/**
 * @brief ROS를 초기화하고 Pick-and-Place 노드를 MultiThreadedExecutor로 실행한다.
 * @param argc 명령행 인자 개수.
 * @param argv ROS 인자를 포함한 명령행 인자 배열.
 * @return 정상 종료 시 0, 노드 생성 또는 실행 중 표준 예외가 발생하면 1.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = createPickAndPlaceNode();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("pick_and_place_test"), "%s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
