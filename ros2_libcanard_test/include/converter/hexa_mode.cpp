#include "hexa_mode.hpp"

HexaModeConverter::HexaModeConverter() 
{
    motor_commands_.setZero();
}

HexaModeConverter::HexaModeConverter(DroneParam drone_param) 
{
    motor_commands_.setZero();
    cmd_to_rpm_converter_ = new CmdToRpmConverter(drone_param);
}

HexaModeConverter::~HexaModeConverter() 
{
    delete cmd_to_rpm_converter_;
}

void HexaModeConverter::set_rc_input(const uint16_t * rc_in_channels) 
{
    cmd_to_rpm_converter_->update_rc_input(rc_in_channels);
    motor_commands_ = cmd_to_rpm_converter_->get_motor_rpms();
}

Vector6i16 HexaModeConverter::get_motor_commands() 
{
    return motor_commands_;
}