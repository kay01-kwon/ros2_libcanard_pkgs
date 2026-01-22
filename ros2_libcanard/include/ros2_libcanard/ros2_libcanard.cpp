#include "ros2_libcanard.hpp"

Ros2Libcanard::Ros2Libcanard()
: Node("ros2_libcanard_node")
{
    int NUM_ESC;
    std::string interface_name;

    NUM_ESC = 6;
    interface_name = "can0";

    // Declare parameters
    this->declare_parameter("interface_name", interface_name);
    this->declare_parameter("num_esc", NUM_ESC);
    this->declare_parameter("low_voltage_threshold", low_voltage_threshold_);
    this->declare_parameter("critical_voltage_threshold", critical_voltage_threshold_);

    // Get parameters from parameter server
    this->get_parameter("interface_name", interface_name);
    this->get_parameter("num_esc", NUM_ESC);
    this->get_parameter("low_voltage_threshold", low_voltage_threshold_);
    this->get_parameter("critical_voltage_threshold", critical_voltage_threshold_);

    printf("Interface: %s\n", interface_name.c_str());
    printf("Number of ESC: %d\n", NUM_ESC);
    printf("Low voltage threshold: %.2f V\n", low_voltage_threshold_);
    printf("Critical voltage threshold: %.2f V\n", critical_voltage_threshold_);

    NUM_ESC_ = static_cast<uint8_t>(NUM_ESC);
    canard_interface_.init(interface_name.c_str());
    uint8_t node_id = canard_interface_.get_node_id();

    // Display the number of ESC and Drone can info
    std::cout<<"Number of ESC from launch file: "<<NUM_ESC<<std::endl;
    printf("Drone Can started on %s, node ID: %d\n",
    interface_name.c_str(),
    canard_interface_.get_node_id());

    send_NodeStatus();
    // Pop Tx queue and socketcanReceive for 10 ms
    canard_interface_.process(10);

    uavcan_protocol_GetNodeInfoRequest req;

    // Request node info for nodes
    for(size_t i = 0; i < NUM_ESC_; i++)
    {
        printf("Requesting node info for node %ld\n",i+2);
        req = {};
        while(!get_node_info_client_.request(i+2, req))
        {
            printf("Pending response\n");
            // Pop Tx queue and socketcanReceive for 10 ms
            canard_interface_.process(10);
        }
    }

    // Switch to Operational Mode
    printf("Switch to Operational mode\n");

    uavcan_cmd_msg_.cmd.len = NUM_ESC_;

    for(size_t i = 0; i < NUM_ESC_; i++)
    {
        uavcan_cmd_msg_.cmd.data[i] = 10;
    }
    // By broadcasting cmd data to esc
    // switch to operational mode
    esc_cmd_pub_.broadcast(uavcan_cmd_msg_);

    if(NUM_ESC_ == 1)
    {
        single_actual_rpm_pub_ = this->create_publisher<SingleActualRpm>("/uav/actual_rpm", 
            rclcpp::SensorDataQoS());
        single_cmd_raw_broadcast_pub_ = this->create_publisher<SingleActualRpm>("/uav/broadcast", 1);
        single_cmd_raw_sub_ = this->create_subscription<SingleCmdRaw>("/uav/cmd_raw", 1,
            std::bind(&Ros2Libcanard::single_cmd_raw_callback, this, std::placeholders::_1));
        
        uav_type_ = UavType::SINGLE;
    }
    else if(NUM_ESC_ == 4)
    {
        quad_actual_rpm_pub_ = this->create_publisher<QuadActualRpm>("/uav/actual_rpm", 
            rclcpp::SensorDataQoS());
        quad_cmd_raw_broadcast_pub_ = this->create_publisher<QuadActualRpm>("/uav/broadcast", 1);
        quad_cmd_raw_sub_ = this->create_subscription<QuadCmdRaw>("/uav/cmd_raw", 1,
            std::bind(&Ros2Libcanard::quad_cmd_raw_callback, this, std::placeholders::_1));

        uav_type_ = UavType::QUAD;
    }
    else if(NUM_ESC_ == 6)
    {
        hexa_actual_rpm_pub_ = this->create_publisher<HexaActualRpm>("/uav/actual_rpm", 
            rclcpp::SensorDataQoS());
        hexa_cmd_raw_broadcast_pub_ = this->create_publisher<HexaActualRpm>("/uav/broadcast", 1);
        hexa_cmd_raw_sub_ = this->create_subscription<HexaCmdRaw>("/uav/cmd_raw", 1,
            std::bind(&Ros2Libcanard::hexa_cmd_raw_callback, this, std::placeholders::_1));
        uav_type_ = UavType::HEXA;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(),"Unsupported number of ESC: %d", NUM_ESC_);
        throw std::runtime_error("Unsupported number of ESC");
    }

    voltage_pub_ = this->create_publisher<Float64>("voltage", 
        rclcpp::SensorDataQoS());

    canard_process_timer_ = this->create_wall_timer(100us,
        std::bind(&Ros2Libcanard::process_canard, this));

    // Delay raw_cmd_timer start by 200ms to allow ESCs to stabilize
    // Especially important with multiple ESCs (6 ESCs case)
    printf("Waiting for ESCs to stabilize (200ms delay)...\n");
    init_timer_ = this->create_wall_timer(200ms,
        std::bind(&Ros2Libcanard::start_raw_cmd_timer, this));

    // node_status_timer_ = this->create_wall_timer(1s,
    //     std::bind(&Ros2Libcanard::send_NodeStatus, this));

}

Ros2Libcanard::~Ros2Libcanard()
{
    
}

void Ros2Libcanard::process_canard()
{
    canard_interface_.process(1);
}

void Ros2Libcanard::single_cmd_raw_callback(const ros2_libcanard_msgs::msg::SingleCmdRaw::SharedPtr msg)
{
    uavcan_cmd_msg_.cmd.len = NUM_ESC_;
    uavcan_cmd_msg_.cmd.data[0] = msg->cmd_raw;
}

void Ros2Libcanard::quad_cmd_raw_callback(const ros2_libcanard_msgs::msg::QuadCmdRaw::SharedPtr msg)
{
    uavcan_cmd_msg_.cmd.len = NUM_ESC_;
    for(int i = 0; i < NUM_ESC_; i++)
    {
        uavcan_cmd_msg_.cmd.data[i] = msg->cmd_raw[i];
    }
}

void Ros2Libcanard::hexa_cmd_raw_callback(const ros2_libcanard_msgs::msg::HexaCmdRaw::SharedPtr msg)
{
    uavcan_cmd_msg_.cmd.len = NUM_ESC_;
    for(int i = 0; i < NUM_ESC_; i++)
    {
        uavcan_cmd_msg_.cmd.data[i] = msg->cmd_raw[i];
    }
}

void Ros2Libcanard::start_raw_cmd_timer()
{
    printf("ESCs ready - starting command timer\n");
    // Cancel the one-shot init timer
    if(init_timer_)
    {
        init_timer_->cancel();
        init_timer_.reset();
    }
    // Start the periodic raw command timer
    raw_cmd_timer_ = this->create_wall_timer(10ms,
        std::bind(&Ros2Libcanard::raw_cmd_timer_callback, this));
}

void Ros2Libcanard::raw_cmd_timer_callback()
{
    bool success = esc_cmd_pub_.broadcast(uavcan_cmd_msg_);

    if(!success)
    {
        broadcast_fail_count_++;
        RCLCPP_WARN(this->get_logger(),"Failed to broadcast ESC command (consecutive failures: %zu)",
            broadcast_fail_count_);
    }
    else
    {
        // Reset counter on successful broadcast
        if(broadcast_fail_count_ > 0)
        {
            RCLCPP_INFO(this->get_logger(),"Broadcast recovered after %zu failures",
                broadcast_fail_count_);
            broadcast_fail_count_ = 0;
        }
    }

    auto single_broadcast_msg = SingleActualRpm();
    auto quad_broadcast_msg = QuadActualRpm();
    auto hexa_broadcast_msg = HexaActualRpm();

    switch(uav_type_)
    {
        case UavType::SINGLE:

            single_broadcast_msg.header.stamp = this->now();
            single_broadcast_msg.rpm = uavcan_cmd_msg_.cmd.data[0]*9800.0/8191.0;
            single_cmd_raw_broadcast_pub_->publish(single_broadcast_msg);
            break;
        case UavType::QUAD:
            quad_broadcast_msg.header.stamp = this->now();
            for(size_t i = 0; i < NUM_ESC_; i++)
            {
                quad_broadcast_msg.rpm[i] = uavcan_cmd_msg_.cmd.data[i]*9800.0/8191.0;
            }
            quad_cmd_raw_broadcast_pub_->publish(quad_broadcast_msg);
            break;
        case UavType::HEXA:
            hexa_broadcast_msg.header.stamp = this->now();
            for(size_t i = 0; i < NUM_ESC_; i++)
            {
                hexa_broadcast_msg.rpm[i] = uavcan_cmd_msg_.cmd.data[i]*9800.0/8191.0;
            }
            hexa_cmd_raw_broadcast_pub_->publish(hexa_broadcast_msg);
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),"Unsupported UAV type");
            return;
    }

}

void Ros2Libcanard::handle_esc_status(const CanardRxTransfer &transfer,
                           const uavcan_equipment_esc_Status &msg)
{

    switch(uav_type_)
    {
        case UavType::SINGLE:
            single_actual_rpm_msg_.rpm = msg.rpm;
            single_actual_rpm_msg_.acceleration = 0;
            break;
        case UavType::QUAD:
            quad_actual_rpm_msg_.rpm[msg.esc_index] = msg.rpm;
            quad_actual_rpm_msg_.acceleration[msg.esc_index] = 0;
            break;
        case UavType::HEXA:
            hexa_actual_rpm_msg_.rpm[msg.esc_index] = msg.rpm;
            hexa_actual_rpm_msg_.acceleration[msg.esc_index] = 0;
            break;
        default:
            RCLCPP_ERROR(this->get_logger(),"Unsupported UAV type");
            return;
    }

    esc_count_++;
    
    if(esc_count_ == NUM_ESC_)
    {
        auto quad_broadcast_msg = QuadActualRpm();
        auto hexa_broadcast_msg = HexaActualRpm();
        switch(uav_type_)
        {
            case UavType::SINGLE:
                single_actual_rpm_msg_.header.stamp = this->now();
                single_actual_rpm_pub_->publish(single_actual_rpm_msg_);
                break;
            case UavType::QUAD:
                quad_actual_rpm_msg_.header.stamp = this->now();
                quad_actual_rpm_pub_->publish(quad_actual_rpm_msg_);
                break;
            case UavType::HEXA:                
                hexa_actual_rpm_msg_.header.stamp = this->now();
                hexa_actual_rpm_pub_->publish(hexa_actual_rpm_msg_);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(),"Unsupported UAV type");
                return;
        }
        
        // esc_cmd_pub_.broadcast(uavcan_cmd_msg_);
        voltage_msg_.data = msg.voltage;
        voltage_pub_->publish(voltage_msg_);

        // Check voltage and update state
        check_voltage_and_update_state(msg.voltage);

        esc_count_ = 0;
    }
}

void Ros2Libcanard::handle_getNodeInfo(const CanardRxTransfer& transfer,
    const uavcan_protocol_GetNodeInfoResponse &rsp)
{
    printf("Got GetNodeInfo response\n");
    printf("ESC Name: ");
    for(int i = 0; i < rsp.name.len; i++)
    {
        printf("%c", rsp.name.data[i]);
    }
    printf("\n");
}

void Ros2Libcanard::send_NodeStatus()
{
    uavcan_node_status_msg_.vendor_specific_status_code = 0;
    uavcan_node_status_msg_.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    uavcan_node_status_msg_.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    uavcan_node_status_msg_.sub_mode = 0;
    uavcan_node_status_msg_.uptime_sec = millis32() / 1000UL;
    node_status_pub_.broadcast(uavcan_node_status_msg_);
}

void Ros2Libcanard::check_voltage_and_update_state(double voltage)
{
    current_voltage_ = voltage;

    // State machine for voltage monitoring
    switch(voltage_state_)
    {
        case VoltageState::NORMAL:
            if(voltage < critical_voltage_threshold_)
            {
                voltage_state_ = VoltageState::CRITICAL_EMERGENCY_LANDING;
                RCLCPP_ERROR(this->get_logger(),
                    "CRITICAL: Battery voltage %.2fV below threshold %.2fV - INITIATING EMERGENCY LANDING",
                    voltage, critical_voltage_threshold_);
                emergency_landing_active_ = true;
            }
            else if(voltage < low_voltage_threshold_)
            {
                voltage_state_ = VoltageState::LOW_VOLTAGE_WARNING;
                RCLCPP_WARN(this->get_logger(),
                    "WARNING: Battery voltage %.2fV below threshold %.2fV - Return to land recommended",
                    voltage, low_voltage_threshold_);
            }
            break;

        case VoltageState::LOW_VOLTAGE_WARNING:
            if(voltage < critical_voltage_threshold_)
            {
                voltage_state_ = VoltageState::CRITICAL_EMERGENCY_LANDING;
                RCLCPP_ERROR(this->get_logger(),
                    "CRITICAL: Battery voltage %.2fV below threshold %.2fV - INITIATING EMERGENCY LANDING",
                    voltage, critical_voltage_threshold_);
                emergency_landing_active_ = true;
            }
            else if(voltage >= low_voltage_threshold_)
            {
                voltage_state_ = VoltageState::NORMAL;
                RCLCPP_INFO(this->get_logger(),
                    "Battery voltage recovered to %.2fV - Normal operation resumed",
                    voltage);
            }
            break;

        case VoltageState::CRITICAL_EMERGENCY_LANDING:
            if(voltage >= low_voltage_threshold_)
            {
                voltage_state_ = VoltageState::NORMAL;
                emergency_landing_active_ = false;
                RCLCPP_INFO(this->get_logger(),
                    "Battery voltage recovered to %.2fV - Emergency landing cancelled",
                    voltage);
            }
            else
            {
                // Continue emergency landing
                apply_emergency_landing();
            }
            break;
    }
}

void Ros2Libcanard::apply_emergency_landing()
{
    // Gradually reduce ESC commands to safe landing speed
    // Reduce by 10% per call (called at ESC status rate ~50Hz for 6 ESCs)
    const double reduction_factor = 0.98;  // 2% reduction per call

    for(size_t i = 0; i < NUM_ESC_; i++)
    {
        if(uavcan_cmd_msg_.cmd.data[i] > 100)  // Keep minimum control authority
        {
            uavcan_cmd_msg_.cmd.data[i] =
                static_cast<int16_t>(uavcan_cmd_msg_.cmd.data[i] * reduction_factor);
        }
    }

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Emergency landing in progress - Current voltage: %.2fV, Reducing ESC commands",
        current_voltage_);
}