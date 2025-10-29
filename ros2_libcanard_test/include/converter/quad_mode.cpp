#include "quad_mode.hpp"
QuadModeConverter::QuadModeConverter() 
{
    motor_commands_.setZero();
}

QuadModeConverter::QuadModeConverter(DroneParam drone_param) 
{
    motor_commands_.setZero();
    cmd_to_rpm_converter_ = new CmdToRpmConverter(drone_param);
}

QuadModeConverter::~QuadModeConverter() 
{
    delete cmd_to_rpm_converter_;
}

void QuadModeConverter::set_rc_input(const uint16_t * rc_in_channels) 
{
    cmd_to_rpm_converter_->update_rc_input(rc_in_channels);
    motor_commands_ = cmd_to_rpm_converter_->get_motor_rpms();
}

Vector6i16 QuadModeConverter::get_motor_commands() 
{
    // Discard the last two channels for quad mode
    return motor_commands_;
}