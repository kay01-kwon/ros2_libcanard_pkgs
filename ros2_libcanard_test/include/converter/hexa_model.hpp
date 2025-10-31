#ifndef HEXA_MODEL_HPP
#define HEXA_MODEL_HPP

#include "rc_converter.hpp"
#include "cmd_to_rpm.hpp"

class HexaModelConverter : public RCConverter
{
    public:

    HexaModelConverter();

    HexaModelConverter(DroneParam drone_param);

    ~HexaModelConverter() override;

    void set_rc_input(const uint16_t* rc_in_channels) override;

    Vector6i16 get_motor_commands() override;

    RCMode get_rc_mode() const override;

    private:
    CmdToRpmConverter *cmd_to_rpm_converter_{nullptr};

};


#endif // HEXA_MODEL_HPP