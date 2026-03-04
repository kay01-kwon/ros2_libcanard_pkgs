#include "rotor_step_test.hpp"

RotorStepTestNode::RotorStepTestNode()
: Node("rotor_step_test_node")
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
        std::bind(&RotorStepTestNode::timer_callback, this)
    );

    start_time_ = this->now();
    step_state_ = StepState::IDLE;

    RCLCPP_INFO(this->get_logger(), "Rotor Step Test Node started");
    RCLCPP_INFO(this->get_logger(), "Test will start after 2 seconds...");
}

RotorStepTestNode::~RotorStepTestNode()
{
    RCLCPP_INFO(this->get_logger(), "Rotor Step Test Node destroyed");
}

void RotorStepTestNode::configure()
{
    // Declare and get parameters from YAML config
    this->declare_parameter("initial_rpm", initial_rpm_);
    this->get_parameter("initial_rpm", initial_rpm_);

    this->declare_parameter("step_rpm", step_rpm_);
    this->get_parameter("step_rpm", step_rpm_);

    this->declare_parameter("step_start_time", step_start_time_);
    this->get_parameter("step_start_time", step_start_time_);

    this->declare_parameter("hold_duration", hold_duration_);
    this->get_parameter("hold_duration", hold_duration_);

    this->declare_parameter("end_time", end_time_);
    this->get_parameter("end_time", end_time_);

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

void RotorStepTestNode::print_status()
{
    RCLCPP_INFO(this->get_logger(), "==== Rotor Step Test Configuration ====");
    RCLCPP_INFO(this->get_logger(), "Drone Model: %s",
        drone_model_ == DroneModel::SINGLE ? "SINGLE" :
        drone_model_ == DroneModel::QUAD ? "QUAD" : "HEXA");
    RCLCPP_INFO(this->get_logger(), "Initial RPM: %.1f", initial_rpm_);
    RCLCPP_INFO(this->get_logger(), "Step RPM: %.1f", step_rpm_);
    RCLCPP_INFO(this->get_logger(), "Step Start Time: %.1f seconds", step_start_time_);
    RCLCPP_INFO(this->get_logger(), "Hold Duration: %.1f seconds", hold_duration_);
    RCLCPP_INFO(this->get_logger(), "End Time: %.1f seconds", end_time_);
    RCLCPP_INFO(this->get_logger(), "=======================================");
}

void RotorStepTestNode::update_step_state()
{
    rclcpp::Time current_time = this->now();
    double elapsed_time = (current_time - start_time_).seconds();

    // Wait 2 seconds before starting
    if (!test_started_ && elapsed_time < 2.0) {
        step_state_ = StepState::IDLE;
        return;
    }

    if (!test_started_) {
        test_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting test at initial RPM...");
    }

    // Update test elapsed time (after the 2 second delay)
    double test_elapsed = elapsed_time - 2.0;

    // State transitions based on test elapsed time
    if (test_elapsed >= end_time_) {
        if (step_state_ != StepState::COMPLETED) {
            step_state_ = StepState::COMPLETED;
            RCLCPP_INFO(this->get_logger(), "Test completed!");
        }
    }
    else if (test_elapsed >= step_start_time_ + hold_duration_) {
        if (step_state_ != StepState::STEP_DOWN) {
            step_state_ = StepState::STEP_DOWN;
            step_down_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Stepping down to initial RPM...");
        }
    }
    else if (test_elapsed >= step_start_time_) {
        if (step_state_ != StepState::STEP_UP) {
            step_state_ = StepState::STEP_UP;
            step_up_time_ = current_time;
            RCLCPP_INFO(this->get_logger(), "Stepping up to %.1f RPM...", step_rpm_);
        }
    }
}

double RotorStepTestNode::calculate_current_rpm()
{
    double rpm = initial_rpm_;

    switch(step_state_)
    {
        case StepState::IDLE:
            rpm = initial_rpm_;
            break;

        case StepState::STEP_UP:
            rpm = step_rpm_;
            break;

        case StepState::STEP_DOWN:
            rpm = initial_rpm_;
            break;

        case StepState::COMPLETED:
            rpm = 0.0;  // Stop completely
            break;
    }

    return rpm;
}

int16_t RotorStepTestNode::rpm_to_cmd(double rpm)
{
    // Clamp RPM to valid range
    rpm = std::max(0.0, std::min(rpm, max_rpm_));

    // Convert RPM to command bit value
    int16_t cmd = static_cast<int16_t>((rpm / max_rpm_) * max_bit_);

    return cmd;
}

void RotorStepTestNode::publish_commands(double rpm)
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

void RotorStepTestNode::timer_callback()
{
    update_step_state();
    double current_rpm = calculate_current_rpm();
    publish_commands(current_rpm);

    // Log current state periodically (every second)
    static rclcpp::Time last_log_time = this->now();
    rclcpp::Time current_time = this->now();
    if ((current_time - last_log_time).seconds() >= 1.0) {
        const char* state_str;
        switch(step_state_) {
            case StepState::IDLE: state_str = "IDLE"; break;
            case StepState::STEP_UP: state_str = "STEP_UP"; break;
            case StepState::STEP_DOWN: state_str = "STEP_DOWN"; break;
            case StepState::COMPLETED: state_str = "COMPLETED"; break;
            default: state_str = "UNKNOWN"; break;
        }
        RCLCPP_INFO(this->get_logger(), "State: %s | RPM: %.1f", state_str, current_rpm);
        last_log_time = current_time;
    }
}
