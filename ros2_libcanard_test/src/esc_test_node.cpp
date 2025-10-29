#include "node/esc_test.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ESCTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}