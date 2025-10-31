#ifndef RC_STATE_DEF_HPP
#define RC_STATE_DEF_HPP

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

typedef Eigen::Matrix<int16_t, 6, 1> Vector6i16;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

typedef Eigen::Matrix<double, 4, 1> Vector4d;

typedef Eigen::Matrix<double, 4, 4> Matrix4x4d;
typedef Eigen::Matrix<double, 4, 6> Matrix4x6d;
typedef Eigen::Matrix<double, 6, 4> Matrix6x4d;

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

enum class RCMode{
    DISARMED,
    ARMED,
    KILL
};

enum class DroneModel
{
    SINGLE,
    QUAD,
    HEXA
};

enum class ThreePosSwitch{
    LOW,
    MID,
    HIGH
};

enum class TwoPosSwitch{
    LOW,
    HIGH
};

struct DroneParam
{
    double motor_const{1.465e-07}; // motor constant (thrust = motor_const * rpm^2)
    double moment_const{0.01569}; // moment constant (moment = moment_const * thrust)
    double arm_length{0.265}; // distance from motor to center of drone
    DroneModel drone_model{DroneModel::HEXA}; // Drone model (SINGLE, QUAD, HEXA)
    double Tmax{9.376}; // maximum thrust (N)
    double Tmin{0.586}; // minimum thrust (N)
};

#endif // RC_STATE_DEF_HPP