#include <rclcpp/rclcpp.hpp>
#include "node/rotor_ramp_test.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotorRampTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
