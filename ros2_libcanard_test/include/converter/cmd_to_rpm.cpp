#include "cmd_to_rpm.hpp"

CmdToRpmConverter::CmdToRpmConverter()
{

}

CmdToRpmConverter::CmdToRpmConverter(const DroneParam& drone_param)
:drone_param_(drone_param)
{
    // Constructor implementation based on RCMode
    // (Details would depend on specific requirements)
    allocate_matrix();
}

CmdToRpmConverter::~CmdToRpmConverter()
{
    // Destructor implementation (if needed)
}

void CmdToRpmConverter::update_rc_input(const uint16_t* rc_in_channels)
{
    // Update motor_commands_ based on rc_in_channels
    // (Details would depend on specific requirements)

    Vector4d control_input;

    // control_input[0]: collective thrust command
    // control_input[1]: roll command
    // control_input[2]: pitch command
    // control_input[3]: yaw command

    control_input(0) = static_cast<double>(
        (rc_in_channels[2] - rc_in_min)
        /(double)(2*rc_in_delta)
        *(drone_param_.Tmax - drone_param_.Tmin)
        *(double) num_rotors_
        + drone_param_.Tmin 
        *(double) num_rotors_
    ); // Throttle

    control_input(1) = static_cast<double>(
        (rc_in_channels[0] - rc_in_mid)
        /(double)(2*rc_in_delta)
        *2.0
    );  // Roll

    control_input(2) = static_cast<double>(
        (rc_in_channels[1] - rc_in_mid)
        /(double)(2*rc_in_delta)
        *2.0
    );  // Pitch
    control_input(3) = static_cast<double>(
        (rc_in_channels[3] - rc_in_mid)
        /(double)(2*rc_in_delta)
        *0.5
    );  // Yaw

    if(drone_param_.rc_mode == RCMode::QUAD)
    {
        Vector4d motor_thrusts = quad_allocation_matrix_ * control_input;

        for(int i = 0; i < 4; ++i)
        {
            thrust_clamp(motor_thrusts(i));
            motor_rpms_(i) = std::sqrt(motor_thrusts(i) / drone_param_.motor_const);
        }
        for(int i = 4; i < 6; ++i)
        {
            motor_rpms_(i) = 0.0;
        }
    }
    else if(drone_param_.rc_mode == RCMode::HEXA)
    {
        Vector6d motor_thrusts = hexa_allocation_matrix_ * control_input;

        for(int i = 0; i < 6; ++i)
        {
            thrust_clamp(motor_thrusts(i));
            motor_rpms_(i) = std::sqrt(motor_thrusts(i) / drone_param_.motor_const);
        }
    }
    else
    {
        std::cerr << "Unsupported RC Mode for updating RC input." << std::endl;
    }

}

Vector6i16 CmdToRpmConverter::get_motor_rpms() const
{
    Vector6i16 motor_rpms;
    for(int i = 0; i < 6; ++i)
    {
        motor_rpms(i) = static_cast<int16_t>(motor_rpms_(i));
    }

    return motor_rpms;
}

void CmdToRpmConverter::allocate_matrix()
{
    // Allocate and initialize quad_allocation_matrix_ based on drone_param
    // (Details would depend on specific requirements)

    double l = drone_param_.arm_length;
    double km = drone_param_.moment_const;


    if(drone_param_.rc_mode == RCMode::QUAD)
    {
        num_rotors_ = 4;
        double cos_pi_4 = std::cos(M_PI / 4.0);
        double sin_pi_4 = std::sin(M_PI / 4.0);

        double lx1, lx2, lx3, lx4;
        double ly1, ly2, ly3, ly4;

        lx1 = l * cos_pi_4;
        lx2 = -l * cos_pi_4;
        lx3 = -l * cos_pi_4;
        lx4 = l * cos_pi_4;

        ly1 = l * sin_pi_4;
        ly2 = l * sin_pi_4;
        ly3 = -l * sin_pi_4;
        ly4 = -l * sin_pi_4;

        Matrix4x4d temp_matrix;
        
        temp_matrix <<
        1.0, 1.0, 1.0, 1.0,
        ly1, ly2, ly3, ly4,
        -lx1, -lx2, -lx3, -lx4,
        -km, km, -km, km;

        // Compute pseudo-inverse using SVD
        Eigen::JacobiSVD<Matrix4x4d> svd(temp_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix4x4d S_inv;

        for(int i = 0; i < svd.rank(); ++i)
        {
            S_inv(i,i) = 1.0 / svd.singularValues()(i);
        }
        quad_allocation_matrix_ = svd.matrixV() * S_inv * svd.matrixU().transpose();
    }
    else if(drone_param_.rc_mode == RCMode::HEXA)
    {
        num_rotors_ = 6;
        double cos_pi_3 = std::cos(M_PI / 3.0);
        double sin_pi_3 = std::sin(M_PI / 3.0);

        double lx1, lx2, lx3, lx4, lx5, lx6;
        double ly1, ly2, ly3, ly4, ly5, ly6;

        lx1 = l*sin_pi_3;
        lx2 = 0.0;
        lx3 = -l*sin_pi_3;
        lx4 = -l*sin_pi_3;
        lx5 = 0.0;
        lx6 = l*sin_pi_3;

        ly1 = l*cos_pi_3;
        ly2 = l;
        ly3 = l*cos_pi_3;
        ly4 = -l*cos_pi_3;
        ly5 = -l;
        ly6 = -l*cos_pi_3;

        Matrix4x6d temp_matrix;

        temp_matrix <<
        1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        ly1, ly2, ly3, ly4, ly5, ly6,
        -lx1, -lx2, -lx3, -lx4, -lx5, -lx6,
        -km, km, -km, km, -km, km;

        // Compute pseudo-inverse using SVD
        Eigen::JacobiSVD<Matrix4x6d> svd(temp_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix6x4d S_inv;

        for(int i = 0; i < svd.rank(); ++i)
        {
            S_inv(i,i) = 1.0 / svd.singularValues()(i);
        }
        hexa_allocation_matrix_ = svd.matrixV() * S_inv * svd.matrixU().transpose();
    }
    else
    {
        std::cerr << "Unsupported RC Mode for allocation matrix." << std::endl;
    }

}

double CmdToRpmConverter::thrust_clamp(double &thrust)
{
    if(thrust > drone_param_.Tmax)
    {
        thrust = drone_param_.Tmax;
    }
    else if(thrust < drone_param_.Tmin)
    {
        thrust = drone_param_.Tmin;
    }
    return thrust;
}