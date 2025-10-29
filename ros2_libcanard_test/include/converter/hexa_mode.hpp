#ifndef HEXA_MODE_HPP
#define HEXA_MODE_HPP

#include "rc_converter.hpp"
#include "cmd_to_rpm.hpp"

class HexaModeConverter : public RCConverter
{
    public:

    HexaModeConverter();

    HexaModeConverter(DroneParam drone_param);

    ~HexaModeConverter() override;

    void set_rc_input(const uint16_t* rc_in_channels) override;

    Vector6i16 get_motor_commands() override;

    private:
    CmdToRpmConverter *cmd_to_rpm_converter_{nullptr};

};


#endif // HEXA_MODE_HPP