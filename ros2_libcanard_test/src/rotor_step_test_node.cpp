#include <rclcpp/rclcpp.hpp>
#include "node/rotor_step_test.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotorStepTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
