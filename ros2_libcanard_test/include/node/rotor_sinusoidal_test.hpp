#ifndef ROTOR_SINUSOIDAL_TEST_HPP
#define ROTOR_SINUSOIDAL_TEST_HPP

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <cmath>

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

enum class SinusoidalState
{
    IDLE,
    RUNNING,
    COMPLETED
};

class RotorSinusoidalTestNode : public rclcpp::Node
{
public:
    RotorSinusoidalTestNode();
    ~RotorSinusoidalTestNode();

private:
    void timer_callback();
    void configure();
    void update_sinusoidal_state();
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
    double center_rpm_{4000.0};         // RPM - Center RPM for sinusoidal wave
    double amplitude_{1000.0};          // RPM - Amplitude of oscillation
    double frequency_{0.5};             // Hz - Frequency of sinusoidal wave
    double test_duration_{20.0};        // seconds - Total test duration
    double initial_delay_{2.0};         // seconds - Delay before starting

    double max_bit_{8191.0};
    double max_rpm_{9800.0};

    DroneModel drone_model_{DroneModel::SINGLE};

    // State machine variables
    SinusoidalState sinusoidal_state_{SinusoidalState::IDLE};
    rclcpp::Time start_time_;
    rclcpp::Time test_start_time_;

    bool test_started_{false};

    // Mathematical constants
    static constexpr double PI = 3.14159265358979323846;
};

#endif // ROTOR_SINUSOIDAL_TEST_HPP
