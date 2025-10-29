#ifndef RC_STATE_DEF_HPP
#define RC_STATE_DEF_HPP

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<int16_t, 6, 1> Vector6i16;

typedef Eigen::Matrix<double, 4, 1> Vector4d;

typedef Eigen::Matrix<double, 6, 4> Matrix6x4d;

typedef Eigen::Matrix<double, 4, 6> Matrix4x6d;

typedef Eigen::Matrix<uint16_t, 16, 1> Vector16u16;

const uint16_t rc_in_max = 2012;
const uint16_t rc_in_min = 988;
const uint16_t rc_in_mid = 1500;
const uint16_t rc_in_delta = 512;

struct RCState
{
    uint8_t rssi;                   // Received Signal Strength Indicator
    Vector16u16 rc_in_channels;     // Raw RC input channels
};


#endif // RC_STATE_DEF_HPP