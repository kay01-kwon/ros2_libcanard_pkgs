#ifndef ESC_TEST_HPP
#define ESC_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include "converter/rc_converter.hpp"
#include "converter/cmd_to_rpm.hpp"

#include <ros2_libcanard_msgs/msg/quad_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/quad_actual_rpm.hpp>

#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>

#include <mavros_msgs/msg/rc_in.hpp>

using namespace std::chrono_literals;
using ros2_libcanard_msgs::msg::QuadCmdRaw;
using ros2_libcanard_msgs::msg::QuadActualRpm;

using ros2_libcanard_msgs::msg::HexaCmdRaw;
using ros2_libcanard_msgs::msg::HexaActualRpm;

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

    void print_status(const DroneParam& drone_param);

    rclcpp::Subscription<RCIn>::SharedPtr rc_subscription_;

    rclcpp::Publisher<QuadCmdRaw>::SharedPtr quad_cmd_publisher_;
    rclcpp::Publisher<QuadActualRpm>::SharedPtr quad_cmd_rpm_publisher_;

    rclcpp::Publisher<HexaCmdRaw>::SharedPtr hexa_cmd_publisher_;
    rclcpp::Publisher<HexaActualRpm>::SharedPtr hexa_cmd_rpm_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    RCConverter* rc_converter_;

    QuadCmdRaw quad_cmd_;
    HexaCmdRaw hexa_cmd_;

    QuadActualRpm quad_cmd_rpm_;
    HexaActualRpm hexa_cmd_rpm_;

    double MaxBit_{8191.0};
    double MaxRpm_{9800.0};

    DroneModel drone_model_;

    RCMode rc_mode_{RCMode::DISARMED};
    RCMode rc_mode_prev_{RCMode::DISARMED};
    
};

#endif // ESC_TEST_HPP