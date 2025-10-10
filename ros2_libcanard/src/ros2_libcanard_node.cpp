#include "ros2_libcanard/ros2_libcanard.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Ros2Libcanard>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}