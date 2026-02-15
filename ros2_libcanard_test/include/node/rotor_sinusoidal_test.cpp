#include "rotor_sinusoidal_test.hpp"

RotorSinusoidalTestNode::RotorSinusoidalTestNode()
: Node("rotor_sinusoidal_test_node")
{
    configure();

    // Create publishers based on drone model
    switch(drone_model_)
    {
        case DroneModel::SINGLE:
            single_cmd_publisher_ = this->create_publisher<SingleCmdRaw>("/uav/cmd_raw", 10);
            single_cmd_rpm_publisher_ = this->create_publisher<SingleActualRpm>("/uav/cmd_rpm", 10);
            RCLCPP_INFO(this->get_logger(), "Single rotor mode initialized");
            break;
        case DroneModel::QUAD:
            quad_cmd_publisher_ = this->create_publisher<QuadCmdRaw>("/uav/cmd_raw", 10);
            quad_cmd_rpm_publisher_ = this->create_publisher<QuadActualRpm>("/uav/cmd_rpm", 10);
            RCLCPP_INFO(this->get_logger(), "Quad rotor mode initialized");
            break;
        case DroneModel::HEXA:
            hexa_cmd_publisher_ = this->create_publisher<HexaCmdRaw>("/uav/cmd_raw", 10);
            hexa_cmd_rpm_publisher_ = this->create_publisher<HexaActualRpm>("/uav/cmd_rpm", 10);
            RCLCPP_INFO(this->get_logger(), "Hexa rotor mode initialized");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown drone model");
            break;
    }

    // Create timer with 10ms period (100Hz)
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&RotorSinusoidalTestNode::timer_callback, this)
    );

    start_time_ = this->now();
    sinusoidal_state_ = SinusoidalState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Rotor Sinusoidal Test Node started");
    RCLCPP_INFO(this->get_logger(), "Test will start after %.1f seconds...", initial_delay_);
}

RotorSinusoidalTestNode::~RotorSinusoidalTestNode()
{
    RCLCPP_INFO(this->get_logger(), "Rotor Sinusoidal Test Node destroyed");
}

void RotorSinusoidalTestNode::configure()
{
    // Declare and get parameters from YAML config
    this->declare_parameter("center_rpm", center_rpm_);
    this->get_parameter("center_rpm", center_rpm_);

    this->declare_parameter("amplitude", amplitude_);
    this->get_parameter("amplitude", amplitude_);

    this->declare_parameter("frequency", frequency_);
    this->get_parameter("frequency", frequency_);

    this->declare_parameter("test_duration", test_duration_);
    this->get_parameter("test_duration", test_duration_);

    this->declare_parameter("initial_delay", initial_delay_);
    this->get_parameter("initial_delay", initial_delay_);

    this->declare_parameter("max_bit", max_bit_);
    this->get_parameter("max_bit", max_bit_);

    this->declare_parameter("max_rpm", max_rpm_);
    this->get_parameter("max_rpm", max_rpm_);

    std::string drone_model_str;
    this->declare_parameter("drone_model", "SINGLE");
    this->get_parameter("drone_model", drone_model_str);

    if(drone_model_str == "SINGLE") {
        drone_model_ = DroneModel::SINGLE;
    } else if (drone_model_str == "QUAD") {
        drone_model_ = DroneModel::QUAD;
    } else if (drone_model_str == "HEXA") {
        drone_model_ = DroneModel::HEXA;
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid drone_model parameter, defaulting to SINGLE");
        drone_model_ = DroneModel::SINGLE;
    }

    print_status();
}

void RotorSinusoidalTestNode::print_status()
{
    RCLCPP_INFO(this->get_logger(), "==== Rotor Sinusoidal Test Configuration ====");
    RCLCPP_INFO(this->get_logger(), "Drone Model: %s",
        drone_model_ == DroneModel::SINGLE ? "SINGLE" :
        drone_model_ == DroneModel::QUAD ? "QUAD" : "HEXA");
    RCLCPP_INFO(this->get_logger(), "Center RPM: %.1f RPM", center_rpm_);
    RCLCPP_INFO(this->get_logger(), "Amplitude: %.1f RPM", amplitude_);
    RCLCPP_INFO(this->get_logger(), "RPM Range: %.1f to %.1f RPM",
        center_rpm_ - amplitude_, center_rpm_ + amplitude_);
    RCLCPP_INFO(this->get_logger(), "Frequency: %.2f Hz (Period: %.2f seconds)",
        frequency_, 1.0 / frequency_);
    RCLCPP_INFO(this->get_logger(), "Test Duration: %.1f seconds", test_duration_);
    RCLCPP_INFO(this->get_logger(), "Number of Cycles: %.2f", test_duration_ * frequency_);
    RCLCPP_INFO(this->get_logger(), "Initial Delay: %.1f seconds", initial_delay_);
    RCLCPP_INFO(this->get_logger(), "=============================================");
}

void RotorSinusoidalTestNode::update_sinusoidal_state()
{
    rclcpp::Time current_time = this->now();
    double elapsed_time = (current_time - start_time_).seconds();

    // Wait for initial delay before starting
    if (!test_started_ && elapsed_time < initial_delay_) {
        sinusoidal_state_ = SinusoidalState::IDLE;
        return;
    }

    if (!test_started_) {
        test_started_ = true;
        test_start_time_ = current_time;
        sinusoidal_state_ = SinusoidalState::RUNNING;
        RCLCPP_INFO(this->get_logger(), "Starting sinusoidal test...");
    }

    switch(sinusoidal_state_)
    {
        case SinusoidalState::IDLE:
            // Already handled above
            break;

        case SinusoidalState::RUNNING:
        {
            double test_elapsed = (current_time - test_start_time_).seconds();
            if (test_elapsed >= test_duration_) {
                sinusoidal_state_ = SinusoidalState::COMPLETED;
                RCLCPP_INFO(this->get_logger(), "Sinusoidal test completed!");
            }
            break;
        }

        case SinusoidalState::COMPLETED:
            // Test is done, send center RPM
            break;
    }
}

double RotorSinusoidalTestNode::calculate_current_rpm()
{
    rclcpp::Time current_time = this->now();
    double rpm = center_rpm_;

    switch(sinusoidal_state_)
    {
        case SinusoidalState::IDLE:
            rpm = center_rpm_;
            break;

        case SinusoidalState::RUNNING:
        {
            double test_elapsed = (current_time - test_start_time_).seconds();
            // Sinusoidal formula: rpm = center_rpm + amplitude * sin(2 * pi * frequency * t)
            rpm = center_rpm_ + amplitude_ * std::sin(2.0 * PI * frequency_ * test_elapsed);
            break;
        }

        case SinusoidalState::COMPLETED:
            rpm = center_rpm_;
            break;
    }

    return rpm;
}

int16_t RotorSinusoidalTestNode::rpm_to_cmd(double rpm)
{
    // Clamp RPM to valid range
    rpm = std::max(0.0, std::min(rpm, max_rpm_));

    // Convert RPM to command bit value
    int16_t cmd = static_cast<int16_t>((rpm / max_rpm_) * max_bit_);

    return cmd;
}

void RotorSinusoidalTestNode::publish_commands(double rpm)
{
    int16_t cmd = rpm_to_cmd(rpm);
    int32_t rpm_int = static_cast<int32_t>(rpm);

    switch(drone_model_)
    {
        case DroneModel::SINGLE:
            if (single_cmd_publisher_ != nullptr) {
                single_cmd_.header.stamp = this->now();
                single_cmd_.cmd_raw = cmd;
                single_cmd_publisher_->publish(single_cmd_);

                single_cmd_rpm_.header.stamp = this->now();
                single_cmd_rpm_.rpm = rpm_int;
                single_cmd_rpm_publisher_->publish(single_cmd_rpm_);
            }
            break;

        case DroneModel::QUAD:
            if (quad_cmd_publisher_ != nullptr) {
                quad_cmd_.header.stamp = this->now();
                for (size_t i = 0; i < 4; ++i) {
                    quad_cmd_.cmd_raw[i] = cmd;
                }
                quad_cmd_publisher_->publish(quad_cmd_);

                quad_cmd_rpm_.header.stamp = this->now();
                for (size_t i = 0; i < 4; ++i) {
                    quad_cmd_rpm_.rpm[i] = rpm_int;
                }
                quad_cmd_rpm_publisher_->publish(quad_cmd_rpm_);
            }
            break;

        case DroneModel::HEXA:
            if (hexa_cmd_publisher_ != nullptr) {
                hexa_cmd_.header.stamp = this->now();
                for (size_t i = 0; i < 6; ++i) {
                    hexa_cmd_.cmd_raw[i] = cmd;
                }
                hexa_cmd_publisher_->publish(hexa_cmd_);

                hexa_cmd_rpm_.header.stamp = this->now();
                for (size_t i = 0; i < 6; ++i) {
                    hexa_cmd_rpm_.rpm[i] = rpm_int;
                }
                hexa_cmd_rpm_publisher_->publish(hexa_cmd_rpm_);
            }
            break;
    }
}

void RotorSinusoidalTestNode::timer_callback()
{
    update_sinusoidal_state();
    double current_rpm = calculate_current_rpm();
    publish_commands(current_rpm);

    // Log current state periodically (every second)
    static rclcpp::Time last_log_time = this->now();
    rclcpp::Time current_time = this->now();
    if ((current_time - last_log_time).seconds() >= 1.0) {
        const char* state_str;
        switch(sinusoidal_state_) {
            case SinusoidalState::IDLE: state_str = "IDLE"; break;
            case SinusoidalState::RUNNING: state_str = "RUNNING"; break;
            case SinusoidalState::COMPLETED: state_str = "COMPLETED"; break;
            default: state_str = "UNKNOWN"; break;
        }

        if (sinusoidal_state_ == SinusoidalState::RUNNING) {
            double test_elapsed = (current_time - test_start_time_).seconds();
            double remaining_time = test_duration_ - test_elapsed;
            RCLCPP_INFO(this->get_logger(), "State: %s | RPM: %.1f | Remaining: %.1fs",
                state_str, current_rpm, remaining_time);
        } else {
            RCLCPP_INFO(this->get_logger(), "State: %s | RPM: %.1f", state_str, current_rpm);
        }
        last_log_time = current_time;
    }
}
