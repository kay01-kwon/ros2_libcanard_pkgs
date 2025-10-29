#include "hexa_mode.hpp"

HexaModeConverter::HexaModeConverter() 
{
    motor_commands_.setZero();
}

HexaModeConverter::~HexaModeConverter() 
{

}

void HexaModeConverter::set_rc_input(const uint16_t * rc_in_channels) 
{

}

Vector6i16 HexaModeConverter::get_motor_commands() 
{
    return motor_commands_;
}