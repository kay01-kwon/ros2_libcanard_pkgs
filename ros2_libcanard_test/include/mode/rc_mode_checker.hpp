#ifndef RC_MODE_CHECKER_HPP
#define RC_MODE_CHECKER_HPP

#include "utils/rc_state_def.hpp"



class RCModeChecker{

    public:

    RCModeChecker();

    ~RCModeChecker();

    RCMode& get_rc_mode(const uint16_t* rc_in_channels);

    private:

    TwoPosSwitch get_two_pos_switch_position(const uint16_t rc_input);
    ThreePosSwitch get_three_pos_switch_position(const uint16_t rc_input);


    const uint16_t switch_low = 1200;
    const uint16_t switch_high = 1700;

};

#endif  // RC_MODE_CHECKER_HPP