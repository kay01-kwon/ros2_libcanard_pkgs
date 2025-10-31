#include "quad_model.hpp"
QuadModelConverter::QuadModelConverter() 
{
    motor_commands_.setZero();
}

QuadModelConverter::QuadModelConverter(DroneParam drone_param) 
{
    motor_commands_.setZero();
    cmd_to_rpm_converter_ = new CmdToRpmConverter(drone_param);
}

QuadModelConverter::~QuadModelConverter() 
{
    delete cmd_to_rpm_converter_;
}

void QuadModelConverter::set_rc_input(const uint16_t * rc_in_channels) 
{
    cmd_to_rpm_converter_->update_rc_input(rc_in_channels);
    motor_commands_ = cmd_to_rpm_converter_->get_motor_rpms();
}

Vector6i16 QuadModelConverter::get_motor_commands() 
{
    // Discard the last two channels for quad mode
    return motor_commands_;
}

RCMode QuadModelConverter::get_rc_mode() const 
{
    return cmd_to_rpm_converter_->get_current_rc_mode();
}