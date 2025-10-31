#ifndef SINGLE_MODEL_HPP
#define SINGLE_MODEL_HPP

#include "rc_converter.hpp"
#include "mode/rc_mode_checker.hpp"

class SingleModelConverter : public RCConverter
{
    public:

    SingleModelConverter();

    ~SingleModelConverter() override;

    void set_rc_input(const uint16_t* rc_in_channels) override;

    Vector6i16 get_motor_commands() override;

    RCMode get_rc_mode() const override;

    private:

    RCModeChecker* rc_mode_checker_;
    RCMode current_rc_mode_;

};


#endif // SINGLE_MODEL_HPP