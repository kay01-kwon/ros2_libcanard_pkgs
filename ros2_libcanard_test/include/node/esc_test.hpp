#ifndef ESC_TEST_HPP
#define ESC_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include "converter/rc_converter.hpp"

#include <ros2_libcanard_msgs/msg/quad_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <mavros_msgs/msg/rc_in.hpp>

using namespace std::chrono_literals;
using ros2_libcanard_msgs::msg::QuadCmdRaw;
using ros2_libcanard_msgs::msg::HexaCmdRaw;
using mavros_msgs::msg::RCIn;

class ESCTestNode : public rclcpp::Node
{
    public:

    ESCTestNode();

    ~ESCTestNode();

    private:

    void rc_callback(const RCIn::SharedPtr msg);

    void timer_callback();

    void configure();

    rclcpp::Subscription<RCIn>::SharedPtr rc_subscription_;

    rclcpp::Publisher<QuadCmdRaw>::SharedPtr quad_cmd_publisher_;
    rclcpp::Publisher<HexaCmdRaw>::SharedPtr hexa_cmd_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    RCConverter* rc_converter_;

    QuadCmdRaw quad_cmd_;
    HexaCmdRaw hexa_cmd_;

    RCMode rc_mode_;
};

#endif // ESC_TEST_HPP