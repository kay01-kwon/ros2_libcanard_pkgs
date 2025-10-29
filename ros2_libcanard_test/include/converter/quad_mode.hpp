#ifndef QUAD_MODE_HPP
#define QUAD_MODE_HPP

#include "rc_converter.hpp"
#include "cmd_to_rpm.hpp"

class QuadModeConverter : public RCConverter 
{
public:
    QuadModeConverter();

    QuadModeConverter(DroneParam drone_param);
    
    ~QuadModeConverter();
    
    void set_rc_input(const uint16_t * rc_in_channels) override;
    
    Vector6i16 get_motor_commands() override;

    private:

    CmdToRpmConverter *cmd_to_rpm_converter_{nullptr};
};



#endif // QUAD_MODE_HPP