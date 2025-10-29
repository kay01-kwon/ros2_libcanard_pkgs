#include "single_mode.hpp"

SingleModeConverter::SingleModeConverter()
{
    motor_commands_.setZero();
}

SingleModeConverter::~SingleModeConverter()
{

}

void SingleModeConverter::set_rc_input(const uint16_t* rc_in_channels)
{
    // For Single Mode, we only consider the throttle channel (usually channel 3)
    // Map the throttle input to motor command
    uint16_t throttle_input = rc_in_channels[2]; // Assuming channel 3 is at index 2

    int16_t rpm_command = static_cast<int16_t>(
        (throttle_input - rc_in_min)/(2.0*rc_in_delta)*(MaxRpm - MinRpm)
        + MinRpm
    );

    if (rpm_command >= MaxRpm)
    {
        rpm_command = MaxRpm;
    }

    motor_commands_(0) = rpm_command*MaxBit/MaxRpm;
}

Vector6i16 SingleModeConverter::get_motor_commands()
{
    return motor_commands_;
}