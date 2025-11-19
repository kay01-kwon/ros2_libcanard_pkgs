#include "node/epos4_test.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Epos4TestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}