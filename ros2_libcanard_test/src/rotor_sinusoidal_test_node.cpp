#include <rclcpp/rclcpp.hpp>
#include "node/rotor_sinusoidal_test.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RotorSinusoidalTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
