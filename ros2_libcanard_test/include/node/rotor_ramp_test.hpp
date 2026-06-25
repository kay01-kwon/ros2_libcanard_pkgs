#ifndef ROTOR_RAMP_TEST_HPP
#define ROTOR_RAMP_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

#include <ros2_libcanard_msgs/msg/single_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/single_actual_rpm.hpp>

#include <ros2_libcanard_msgs/msg/quad_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/quad_actual_rpm.hpp>

#include <ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp>
#include <ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp>

#include "utils/rc_state_def.hpp"

using namespace std::chrono_literals;

using ros2_libcanard_msgs::msg::SingleCmdRaw;
using ros2_libcanard_msgs::msg::SingleActualRpm;

using ros2_libcanard_msgs::msg::QuadCmdRaw;
using ros2_libcanard_msgs::msg::QuadActualRpm;

using ros2_libcanard_msgs::msg::HexaCmdRaw;
using ros2_libcanard_msgs::msg::HexaActualRpm;

enum class RampState
{
    IDLE,
    RAMP_UP,
    HOLD,
    RAMP_DOWN,
    COMPLETED
};

class RotorRampTestNode : public rclcpp::Node
{
public:
    RotorRampTestNode();
    ~RotorRampTestNode();

private:
    void timer_callback();
    void configure();
    void update_ramp_state();
    double calculate_current_rpm();
    int16_t rpm_to_cmd(double rpm);
    void publish_commands(double rpm);
    void print_status();

    // Publishers
    rclcpp::Publisher<SingleCmdRaw>::SharedPtr single_cmd_publisher_{nullptr};
    rclcpp::Publisher<SingleActualRpm>::SharedPtr single_cmd_rpm_publisher_{nullptr};

    rclcpp::Publisher<QuadCmdRaw>::SharedPtr quad_cmd_publisher_{nullptr};
    rclcpp::Publisher<QuadActualRpm>::SharedPtr quad_cmd_rpm_publisher_{nullptr};

    rclcpp::Publisher<HexaCmdRaw>::SharedPtr hexa_cmd_publisher_{nullptr};
    rclcpp::Publisher<HexaActualRpm>::SharedPtr hexa_cmd_rpm_publisher_{nullptr};

    rclcpp::TimerBase::SharedPtr timer_;

    // Message objects
    SingleCmdRaw single_cmd_;
    SingleActualRpm single_cmd_rpm_;
    QuadCmdRaw quad_cmd_;
    QuadActualRpm quad_cmd_rpm_;
    HexaCmdRaw hexa_cmd_;
    HexaActualRpm hexa_cmd_rpm_;

    // Configuration parameters
    double idle_rotor_speed_{2000.0};   // RPM
    double max_rotor_speed_{5000.0};    // RPM
    double ramp_up_duration_{5.0};      // seconds
    double hold_duration_{3.0};         // seconds
    double ramp_down_duration_{5.0};    // seconds

    double max_bit_{8191.0};
    double max_rpm_{9800.0};

    DroneModel drone_model_{DroneModel::SINGLE};

    // State machine variables
    RampState ramp_state_{RampState::IDLE};
    rclcpp::Time start_time_;
    rclcpp::Time ramp_up_start_time_;
    rclcpp::Time hold_start_time_;
    rclcpp::Time ramp_down_start_time_;

    bool test_started_{false};
};

#endif // ROTOR_RAMP_TEST_HPP
