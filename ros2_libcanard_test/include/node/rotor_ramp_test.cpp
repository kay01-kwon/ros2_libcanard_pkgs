#include "rotor_ramp_test.hpp"

RotorRampTestNode::RotorRampTestNode()
: Node("rotor_ramp_test_node")
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
        std::bind(&RotorRampTestNode::timer_callback, this)
    );

    start_time_ = this->now();
    ramp_state_ = RampState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Rotor Ramp Test Node started");
    RCLCPP_INFO(this->get_logger(), "Test will start after 2 seconds...");
}

RotorRampTestNode::~RotorRampTestNode()
{
    RCLCPP_INFO(this->get_logger(), "Rotor Ramp Test Node destroyed");
}

void RotorRampTestNode::configure()
{
    // Declare and get parameters from YAML config
    this->declare_parameter("idle_rotor_speed", idle_rotor_speed_);
    this->get_parameter("idle_rotor_speed", idle_rotor_speed_);

    this->declare_parameter("max_rotor_speed", max_rotor_speed_);
    this->get_parameter("max_rotor_speed", max_rotor_speed_);

    this->declare_parameter("ramp_up_duration", ramp_up_duration_);
    this->get_parameter("ramp_up_duration", ramp_up_duration_);

    this->declare_parameter("hold_duration", hold_duration_);
    this->get_parameter("hold_duration", hold_duration_);

    this->declare_parameter("ramp_down_duration", ramp_down_duration_);
    this->get_parameter("ramp_down_duration", ramp_down_duration_);

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

void RotorRampTestNode::print_status()
{
    RCLCPP_INFO(this->get_logger(), "==== Rotor Ramp Test Configuration ====");
    RCLCPP_INFO(this->get_logger(), "Drone Model: %s",
        drone_model_ == DroneModel::SINGLE ? "SINGLE" :
        drone_model_ == DroneModel::QUAD ? "QUAD" : "HEXA");
    RCLCPP_INFO(this->get_logger(), "Idle Rotor Speed: %.1f RPM", idle_rotor_speed_);
    RCLCPP_INFO(this->get_logger(), "Max Rotor Speed: %.1f RPM", max_rotor_speed_);
    RCLCPP_INFO(this->get_logger(), "Ramp Up Duration: %.1f seconds", ramp_up_duration_);
    RCLCPP_INFO(this->get_logger(), "Hold Duration: %.1f seconds", hold_duration_);
    RCLCPP_INFO(this->get_logger(), "Ramp Down Duration: %.1f seconds", ramp_down_duration_);

    double total_duration = ramp_up_duration_ + hold_duration_ + ramp_down_duration_;
    RCLCPP_INFO(this->get_logger(), "Total Test Duration: %.1f seconds", total_duration);
    RCLCPP_INFO(this->get_logger(), "=======================================");
}

void RotorRampTestNode::update_ramp_state()
{
    rclcpp::Time current_time = this->now();
    double elapsed_time = (current_time - start_time_).seconds();

    // Wait 2 seconds before starting
    if (!test_started_ && elapsed_time < 2.0) {
        ramp_state_ = RampState::IDLE;
        return;
    }

    if (!test_started_) {
        test_started_ = true;
        ramp_up_start_time_ = current_time;
        ramp_state_ = RampState::RAMP_UP;
        RCLCPP_INFO(this->get_logger(), "Starting ramp up phase...");
    }

    switch(ramp_state_)
    {
        case RampState::IDLE:
            // Already handled above
            break;

        case RampState::RAMP_UP:
        {
            double ramp_up_elapsed = (current_time - ramp_up_start_time_).seconds();
            if (ramp_up_elapsed >= ramp_up_duration_) {
                hold_start_time_ = current_time;
                ramp_state_ = RampState::HOLD;
                RCLCPP_INFO(this->get_logger(), "Reached max speed, holding...");
            }
            break;
        }

        case RampState::HOLD:
        {
            double hold_elapsed = (current_time - hold_start_time_).seconds();
            if (hold_elapsed >= hold_duration_) {
                ramp_down_start_time_ = current_time;
                ramp_state_ = RampState::RAMP_DOWN;
                RCLCPP_INFO(this->get_logger(), "Starting ramp down phase...");
            }
            break;
        }

        case RampState::RAMP_DOWN:
        {
            double ramp_down_elapsed = (current_time - ramp_down_start_time_).seconds();
            if (ramp_down_elapsed >= ramp_down_duration_) {
                ramp_state_ = RampState::COMPLETED;
                RCLCPP_INFO(this->get_logger(), "Test completed!");
            }
            break;
        }

        case RampState::COMPLETED:
            // Test is done, send idle speed
            break;
    }
}

double RotorRampTestNode::calculate_current_rpm()
{
    rclcpp::Time current_time = this->now();
    double rpm = idle_rotor_speed_;

    switch(ramp_state_)
    {
        case RampState::IDLE:
            rpm = idle_rotor_speed_;
            break;

        case RampState::RAMP_UP:
        {
            double ramp_up_elapsed = (current_time - ramp_up_start_time_).seconds();
            double progress = ramp_up_elapsed / ramp_up_duration_;
            progress = std::min(progress, 1.0);
            rpm = idle_rotor_speed_ + (max_rotor_speed_ - idle_rotor_speed_) * progress;
            break;
        }

        case RampState::HOLD:
            rpm = max_rotor_speed_;
            break;

        case RampState::RAMP_DOWN:
        {
            double ramp_down_elapsed = (current_time - ramp_down_start_time_).seconds();
            double progress = ramp_down_elapsed / ramp_down_duration_;
            progress = std::min(progress, 1.0);
            rpm = max_rotor_speed_ - (max_rotor_speed_ - idle_rotor_speed_) * progress;
            break;
        }

        case RampState::COMPLETED:
            rpm = idle_rotor_speed_;
            break;
    }

    return rpm;
}

int16_t RotorRampTestNode::rpm_to_cmd(double rpm)
{
    // Clamp RPM to valid range
    rpm = std::max(0.0, std::min(rpm, max_rpm_));

    // Convert RPM to command bit value
    int16_t cmd = static_cast<int16_t>((rpm / max_rpm_) * max_bit_);

    return cmd;
}

void RotorRampTestNode::publish_commands(double rpm)
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

void RotorRampTestNode::timer_callback()
{
    update_ramp_state();
    double current_rpm = calculate_current_rpm();
    publish_commands(current_rpm);

    // Log current state periodically (every second)
    static rclcpp::Time last_log_time = this->now();
    rclcpp::Time current_time = this->now();
    if ((current_time - last_log_time).seconds() >= 1.0) {
        const char* state_str;
        switch(ramp_state_) {
            case RampState::IDLE: state_str = "IDLE"; break;
            case RampState::RAMP_UP: state_str = "RAMP_UP"; break;
            case RampState::HOLD: state_str = "HOLD"; break;
            case RampState::RAMP_DOWN: state_str = "RAMP_DOWN"; break;
            case RampState::COMPLETED: state_str = "COMPLETED"; break;
            default: state_str = "UNKNOWN"; break;
        }
        RCLCPP_INFO(this->get_logger(), "State: %s | RPM: %.1f", state_str, current_rpm);
        last_log_time = current_time;
    }
}
