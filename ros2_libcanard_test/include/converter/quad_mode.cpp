#include "quad_mode.hpp"
QuadModeConverter::QuadModeConverter() 
{
    motor_commands_.setZero();
}

QuadModeConverter::~QuadModeConverter() 
{

}

void QuadModeConverter::set_rc_input(const uint16_t * rc_in_channels) 
{

}

Vector6i16 QuadModeConverter::get_motor_commands() 
{
    // Discard the last two channels for quad mode
    return motor_commands_;
}