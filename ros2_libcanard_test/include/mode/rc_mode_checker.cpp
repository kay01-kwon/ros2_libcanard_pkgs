#include "rc_mode_checker.hpp"

RCModeChecker::RCModeChecker()
{

}

RCModeChecker::~RCModeChecker()
{

}

RCMode& RCModeChecker::get_rc_mode(const uint16_t* rc_in_channels)
{

    static RCMode current_state = RCMode::DISARMED;


    // SD switch on channel 8 (index 7)

    TwoPosSwitch is_armed = get_two_pos_switch_position(rc_in_channels[7]);

    // SD HIGH  -> ARMED
    // SD LOW   -> DISARMED
    if ( is_armed == TwoPosSwitch::HIGH)
    {
        current_state = RCMode::ARMED;
        // SF switch on channel 9 (index 8)
        // SF LOW  -> KILL
        // SF HIGH -> normal operation
        TwoPosSwitch deactivate_kill = get_two_pos_switch_position(rc_in_channels[8]);
        if ( deactivate_kill == TwoPosSwitch::LOW)
        {
            current_state = RCMode::KILL;
        }
        else
        {
            // stay in ARMED
        }
    }
    else
    {
        current_state = RCMode::DISARMED;
    }

    return current_state;
}

TwoPosSwitch RCModeChecker::get_two_pos_switch_position(const uint16_t rc_in_channel_value)
{
    // assuming rc_in_channel_value is in range [1000, 2000]
    // LOW  -> [1000, 1499]
    // HIGH -> [1500, 2000]
    if (rc_in_channel_value < 1500)
    {
        return TwoPosSwitch::LOW;
    }
    else
    {
        return TwoPosSwitch::HIGH;
    }
}

ThreePosSwitch RCModeChecker::get_three_pos_switch_position(const uint16_t rc_in_channel_value)
{
    // assuming rc_in_channel_value is in range [1000, 2000]
    // LOW    -> [1000, 1333]
    // MID    -> [1334, 1666]
    // HIGH   -> [1667, 2000]
    if (rc_in_channel_value < 1334)
    {
        return ThreePosSwitch::LOW;
    }
    else if (rc_in_channel_value < 1667)
    {
        return ThreePosSwitch::MID;
    }
    else
    {
        return ThreePosSwitch::HIGH;
    }
}