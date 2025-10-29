#ifndef RC_CONVERTER_HPP
#define RC_CONVERTER_HPP

#include <iostream>
#include "utils/rc_state_def.hpp"
#include "utils/esc_def.hpp"

// Factory method to create RCConverter instances
class RCConverter
{
    public:

    RCConverter() = default;

    static RCConverter* create_RCConverter(DroneParam drone_param);

    virtual ~RCConverter();

    // Do not implement the methods in the base class
    virtual void set_rc_input(const uint16_t* rc_in_channels) = 0;

    virtual Vector6i16 get_motor_commands() = 0;

    protected:

    Vector6i16 motor_commands_;

};

#endif // RC_CONVERTER_HPP