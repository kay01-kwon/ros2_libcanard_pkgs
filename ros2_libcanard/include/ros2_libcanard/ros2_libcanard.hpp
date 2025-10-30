#ifndef ROS2_LIBCANARD__ROS2_LIBCANARD_HPP_
#define ROS2_LIBCANARD__ROS2_LIBCANARD_HPP_

// CANRARD Interface and dronecan DSDL messages
#include "canard_impl/canard_interface/canard_interface.hpp"
#include "dsdl_generated/dronecan_msgs.h"

#include <iostream>

#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ros2_libcanard_msgs/msg/hexa_actual_rpm.hpp"
#include "ros2_libcanard_msgs/msg/hexa_cmd_raw.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

using ros2_libcanard_msgs::msg::HexaActualRpm;
using ros2_libcanard_msgs::msg::HexaCmdRaw;
using std_msgs::msg::Float64;


class Ros2Libcanard : public rclcpp::Node
{
public:
    Ros2Libcanard();

    ~Ros2Libcanard();

private:

    void process_canard();

    void hexa_cmd_raw_callback(const ros2_libcanard_msgs::msg::HexaCmdRaw::SharedPtr msg);


    CanardInterface canard_interface_{0};
    uint8_t NUM_ESC_{6};

    // 1. Declare CANARD publisher
    Canard::Publisher<uavcan_protocol_NodeStatus> 
    node_status_pub_{canard_interface_};
    Canard::Publisher<uavcan_equipment_esc_RawCommand> 
    esc_cmd_pub_{canard_interface_};

    // 2. ESC status
    
    // 2.1 Handler for ESC status
    void handle_esc_status(const CanardRxTransfer &transfer,
                           const uavcan_equipment_esc_Status &msg);
    // 2.2 Object callback for ESC status
    Canard::ObjCallback<Ros2Libcanard, uavcan_equipment_esc_Status> 
    esc_status_cb_{this, &Ros2Libcanard::handle_esc_status};

    // 2.3 Subscriber for ESC status
    Canard::Subscriber<uavcan_equipment_esc_Status> 
    esc_status_sub_{esc_status_cb_, 0};

    // 3. Node handler
    
    // 3.1 Handler for getNodeInfo
    void handle_getNodeInfo(const CanardRxTransfer& transfer,
    const uavcan_protocol_GetNodeInfoResponse &rsp);

    // 3.2 Object callback for getNodeInfo
    Canard::ObjCallback<Ros2Libcanard, uavcan_protocol_GetNodeInfoResponse>
    get_node_info_cb_{this, &Ros2Libcanard::handle_getNodeInfo};

    // 3.3 Declare Client
    Canard::Client<uavcan_protocol_GetNodeInfoResponse>
    get_node_info_client_{canard_interface_, get_node_info_cb_};

    // 4. Member function for Libcanard
    
    void send_NodeStatus();

    uavcan_equipment_esc_RawCommand uavcan_cmd_msg_;
    uavcan_protocol_NodeStatus uavcan_node_status_msg_;
    
    rclcpp::Publisher<HexaActualRpm>::SharedPtr actual_rpm_pub_;
    rclcpp::Publisher<Float64>::SharedPtr voltage_pub_;
    
    HexaActualRpm actual_rpm_msg_;
    Float64 voltage_msg_;

    rclcpp::Subscription<HexaCmdRaw>::SharedPtr cmd_raw_sub_;

    rclcpp::TimerBase::SharedPtr node_status_timer_;
    rclcpp::TimerBase::SharedPtr canard_process_timer_;


    size_t esc_count_{0};
};


#endif  // ROS2_LIBCANARD__ROS2_LIBCANARD_HPP_