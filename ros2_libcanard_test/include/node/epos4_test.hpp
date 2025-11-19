#ifndef EPOS4_TEST_HPP_
#define EPOS4_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include "ros2_libcanard_msgs/msg/single_cmd_raw.hpp"
#include "ros2_libcanard_msgs/msg/single_actual_rpm.hpp"

using namespace std::chrono_literals;
using ros2_libcanard_msgs::msg::SingleCmdRaw;
using ros2_libcanard_msgs::msg::SingleActualRpm;

class Epos4TestNode : public rclcpp::Node
{
public:
    Epos4TestNode();
    ~Epos4TestNode();
private:
    void timer_callback();
    void trajectory_generation();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<SingleCmdRaw>::SharedPtr cmd_publisher_;
    rclcpp::Publisher<SingleActualRpm>::SharedPtr cmd_rpm_publisher_;
    SingleCmdRaw cmd_msg_;
    SingleActualRpm cmd_rpm_msg_;

    double t_now_{0.0};
    double t_start_{5.0};
    double t_offset_{0.0};

    double acc_max_{15000.0}; // Maximum acceleration
    double vel_max_{8000.0}; // Maximum velocity
    double rpm_init_{2000.0};
    double rpm_cmd_{2000.0}; // Commanded RPM

    double t_acc_{0.0};
    double t_cruise_{1.0};

    bool first_run_{false};
};

#endif  // EPOS4_TEST_HPP_