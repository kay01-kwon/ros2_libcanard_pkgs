#include "esc_test.hpp"

ESCTestNode::ESCTestNode()
: Node("esc_test_node")
{
    configure();

    rc_subscription_ = this->create_subscription<RCIn>(
        "/mavros/rc/in",
        rclcpp::SensorDataQoS(),
        std::bind(&ESCTestNode::rc_callback, this, std::placeholders::_1)
    );

    switch(rc_mode_)
    {
        case RCMode::SINGLE:
            quad_cmd_publisher_ = nullptr;
            hexa_cmd_publisher_ = this->create_publisher<HexaCmdRaw>("/uav/cmd_raw", 5);
            break;
        case RCMode::QUAD:
            quad_cmd_publisher_ = this->create_publisher<QuadCmdRaw>("/uav/cmd_raw", 5);
            hexa_cmd_publisher_ = nullptr;
            break;
        case RCMode::HEXA:
            quad_cmd_publisher_ = nullptr;
            hexa_cmd_publisher_ = this->create_publisher<HexaCmdRaw>("/uav/cmd_raw", 5);
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Unknown RC Mode");
            break;
    }

    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&ESCTestNode::timer_callback, this)
    );
}

ESCTestNode::~ESCTestNode()
{
    if (rc_converter_ != nullptr) {
        delete rc_converter_;
    }
}

void ESCTestNode::rc_callback(const RCIn::SharedPtr msg)
{
    const uint16_t* rc_in_channels = msg->channels.data();
    rc_converter_->set_rc_input(rc_in_channels);
    
    Vector6i16 motor_commands = rc_converter_->get_motor_commands();
    
    if (rc_mode_ == RCMode::SINGLE) {
        if (hexa_cmd_publisher_ != nullptr) {
            for (size_t i = 0; i < 6; ++i) {
                hexa_cmd_.cmd_raw[i] = motor_commands(i);
            }
        }
    } else if (rc_mode_ == RCMode::QUAD) {
        if (quad_cmd_publisher_ != nullptr) {
            for (size_t i = 0; i < 4; ++i) {
                quad_cmd_.cmd_raw[i] = motor_commands(i);
            }
        }
    } else if (rc_mode_ == RCMode::HEXA) {
        if (hexa_cmd_publisher_ != nullptr) {
            for (size_t i = 0; i < 6; ++i) {
                hexa_cmd_.cmd_raw[i] = motor_commands(i);
            }
        }
    }

}

void ESCTestNode::timer_callback()
{
    if (rc_mode_ == RCMode::SINGLE) {
        if (hexa_cmd_publisher_ != nullptr) {
            hexa_cmd_publisher_->publish(hexa_cmd_);
        }
    } else if (rc_mode_ == RCMode::QUAD) {
        if (quad_cmd_publisher_ != nullptr) {
            quad_cmd_publisher_->publish(quad_cmd_);
        }
    } else if (rc_mode_ == RCMode::HEXA) {
        if (hexa_cmd_publisher_ != nullptr) {
            hexa_cmd_publisher_->publish(hexa_cmd_);
        }
    }

}

void ESCTestNode::configure()
{
    DroneParam drone_param;
    std::string rc_mode_name;

    this->declare_parameter("rc_mode", "SINGLE");
    this->get_parameter("rc_mode", rc_mode_name);

    if(rc_mode_name == "SINGLE") {
        rc_mode_ = RCMode::SINGLE;
    } else if (rc_mode_name == "QUAD") {
        rc_mode_ = RCMode::QUAD;
    } else if (rc_mode_name == "HEXA") {
        rc_mode_ = RCMode::HEXA;
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid rc_mode parameter, defaulting to SINGLE");
        rc_mode_ = RCMode::SINGLE;
    }

    this->declare_parameter("motor_const", drone_param.motor_const);
    this->get_parameter("motor_const", drone_param.motor_const);

    this->declare_parameter("moment_const", drone_param.moment_const);
    this->get_parameter("moment_const", drone_param.moment_const);

    this->declare_parameter("arm_length", drone_param.arm_length);
    this->get_parameter("arm_length", drone_param.arm_length);

    this->declare_parameter("Tmax", drone_param.Tmax);
    this->get_parameter("Tmax", drone_param.Tmax);

    this->declare_parameter("Tmin", drone_param.Tmin);
    this->get_parameter("Tmin", drone_param.Tmin);

    drone_param.rc_mode = rc_mode_;


    if (rc_converter_ != nullptr) {
        delete rc_converter_;
    }

    rc_converter_ = RCConverter::create_RCConverter(drone_param);

    print_status(drone_param);
}

void ESCTestNode::print_status(const DroneParam& drone_param)
{
    RCLCPP_INFO(this->get_logger(), "---- Drone Parameters ----");

    if(drone_param.rc_mode == RCMode::SINGLE) {
        RCLCPP_INFO(this->get_logger(), "RC Mode: SINGLE");
    } else if (drone_param.rc_mode == RCMode::QUAD) {
        RCLCPP_INFO(this->get_logger(), "RC Mode: QUAD");
    } else if (drone_param.rc_mode == RCMode::HEXA) {
        RCLCPP_INFO(this->get_logger(), "RC Mode: HEXA");
    }

    RCLCPP_INFO(this->get_logger(), "Motor Const: %.8e", drone_param.motor_const);
    RCLCPP_INFO(this->get_logger(), "Moment Const: %f", drone_param.moment_const);
    RCLCPP_INFO(this->get_logger(), "Arm Length: %f", drone_param.arm_length);
    RCLCPP_INFO(this->get_logger(), "Tmax: %f", drone_param.Tmax);
    RCLCPP_INFO(this->get_logger(), "Tmin: %f", drone_param.Tmin);
    RCLCPP_INFO(this->get_logger(), "--------------------------");
}