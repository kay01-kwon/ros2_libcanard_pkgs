#ifndef CMD_TO_RPM_HPP
#define CMD_TO_RPM_HPP

#include "rc_converter.hpp"

class CmdToRpmConverter
{
    public:

    CmdToRpmConverter();

    CmdToRpmConverter(const RCMode &mode);

    ~CmdToRpmConverter();

    void update_rc_input(const uint16_t* rc_in_channels);

    private:

    Vector6i16 motor_commands_;
};


#endif // CMD_TO_RPM_HPP