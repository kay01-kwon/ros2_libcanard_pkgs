#include "hexa_model.hpp"

HexaModelConverter::HexaModelConverter() 
{
    motor_commands_.setZero();
}

HexaModelConverter::HexaModelConverter(DroneParam drone_param) 
{
    motor_commands_.setZero();
    cmd_to_rpm_converter_ = new CmdToRpmConverter(drone_param);
}

HexaModelConverter::~HexaModelConverter() 
{
    delete cmd_to_rpm_converter_;
}

void HexaModelConverter::set_rc_input(const uint16_t * rc_in_channels) 
{
    cmd_to_rpm_converter_->update_rc_input(rc_in_channels);
    motor_commands_ = cmd_to_rpm_converter_->get_motor_rpms();
}

Vector6i16 HexaModelConverter::get_motor_commands() 
{
    return motor_commands_;
}

RCMode HexaModelConverter::get_rc_mode() const
{
    return cmd_to_rpm_converter_->get_current_rc_mode();
}