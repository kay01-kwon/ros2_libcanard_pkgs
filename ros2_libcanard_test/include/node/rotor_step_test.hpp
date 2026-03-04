#ifndef ROTOR_STEP_TEST_HPP
#define ROTOR_STEP_TEST_HPP

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

enum class StepState
{
    IDLE,           // Initial speed before step
    STEP_UP,        // Stepped up to step_rpm
    STEP_DOWN,      // Returned to initial speed
    COMPLETED       // Test finished
};

class RotorStepTestNode : public rclcpp::Node
{
public:
    RotorStepTestNode();
    ~RotorStepTestNode();

private:
    void timer_callback();
    void configure();
    void update_step_state();
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
    double initial_rpm_{2000.0};        // RPM - baseline speed
    double step_rpm_{4000.0};           // RPM - step up speed
    double step_start_time_{5.0};       // seconds - when to step up
    double hold_duration_{10.0};        // seconds - how long to hold at step_rpm
    double end_time_{20.0};             // seconds - when to end test

    double max_bit_{8191.0};
    double max_rpm_{9800.0};

    DroneModel drone_model_{DroneModel::SINGLE};

    // State machine variables
    StepState step_state_{StepState::IDLE};
    rclcpp::Time start_time_;
    rclcpp::Time step_up_time_;
    rclcpp::Time step_down_time_;

    bool test_started_{false};
};

#endif // ROTOR_STEP_TEST_HPP
