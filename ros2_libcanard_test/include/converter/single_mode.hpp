#ifndef SINGLE_MODE_HPP
#define SINGLE_MODE_HPP

#include "rc_converter.hpp"

class SingleModeConverter : public RCConverter
{
    public:

    SingleModeConverter();

    ~SingleModeConverter() override;

    void set_rc_input(const uint16_t* rc_in_channels) override;

    Vector6i16 get_motor_commands() override;

};


#endif // SINGLE_MODE_HPP