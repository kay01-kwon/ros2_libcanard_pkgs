#ifndef CMD_TO_RPM_HPP
#define CMD_TO_RPM_HPP

#include "utils/rc_state_def.hpp"
#include "utils/esc_def.hpp"

class CmdToRpmConverter
{
    public:

    CmdToRpmConverter();

    CmdToRpmConverter(const DroneParam& drone_param);

    ~CmdToRpmConverter();

    void update_rc_input(const uint16_t* rc_in_channels);

    Vector6i16 get_motor_rpms() const;

    private:
    
    void allocate_matrix();

    double thrust_clamp(double &thrust);

    DroneParam drone_param_;

    Vector6d motor_rpms_;

    Matrix4x4d quad_allocation_matrix_;
    Matrix6x4d hexa_allocation_matrix_;

    int num_rotors_{6};

};


#endif // CMD_TO_RPM_HPP