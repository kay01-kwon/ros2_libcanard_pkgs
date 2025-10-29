#include "node/single_esc_test.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleESCTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}