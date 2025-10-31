#include "single_model.hpp"

SingleModelConverter::SingleModelConverter()
{
    motor_commands_.setZero();
    rc_mode_checker_ = new RCModeChecker();
}

SingleModelConverter::~SingleModelConverter()
{
    if (rc_mode_checker_ != nullptr)
    {
        delete rc_mode_checker_;
        rc_mode_checker_ = nullptr;
    }
}

void SingleModelConverter::set_rc_input(const uint16_t* rc_in_channels)
{
    // For Single Mode, we only consider the throttle channel (usually channel 3)
    // Map the throttle input to motor command

    current_rc_mode_ = rc_mode_checker_->get_rc_mode(rc_in_channels);

    if(current_rc_mode_!= RCMode::ARMED)
    {
        motor_commands_.setZero();
        return;
    }

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

Vector6i16 SingleModelConverter::get_motor_commands()
{
    return motor_commands_;
}

RCMode SingleModelConverter::get_rc_mode() const
{
    return current_rc_mode_;
}