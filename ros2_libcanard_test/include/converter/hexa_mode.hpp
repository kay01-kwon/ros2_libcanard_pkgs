#ifndef HEXA_MODE_HPP
#define HEXA_MODE_HPP

#include "rc_converter.hpp"

class HexaModeConverter : public RCConverter
{
    public:

    HexaModeConverter();

    ~HexaModeConverter() override;

    void set_rc_input(const uint16_t* rc_in_channels) override;

    Vector6i16 get_motor_commands() override;

};


#endif // HEXA_MODE_HPP