#include "epos4_test.hpp"

Epos4TestNode::Epos4TestNode()
: Node("epos4_test_node")
{
    cmd_publisher_ = this->create_publisher<SingleCmdRaw>("/uav/cmd_raw", 1);
        cmd_rpm_publisher_ = this->create_publisher<SingleActualRpm>("/uav/cmd_rpm", 1);
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&Epos4TestNode::timer_callback, this)
    );


    t_acc_ = (vel_max_ - rpm_init_) / acc_max_;
}

Epos4TestNode::~Epos4TestNode()
{
}

void Epos4TestNode::timer_callback()
{
    trajectory_generation();
    cmd_publisher_->publish(cmd_msg_);
    cmd_rpm_publisher_->publish(cmd_rpm_msg_);
}

void Epos4TestNode::trajectory_generation()
{
    // Placeholder for trajectory generation logic
    if(!first_run_) {
        t_offset_ = this->now().seconds();
        first_run_ = true;
    }

    t_now_ = this->now().seconds() - t_offset_;
    RCLCPP_INFO(this->get_logger(), "t_now_: %.3f", t_now_);


    if(t_now_ < t_start_)
    {
        rpm_cmd_ = rpm_init_;
    }
    else if(t_now_ < (t_start_ + t_acc_))
    {
        RCLCPP_INFO(this->get_logger(),
        "Acceleration phase: rpm_cmd_ = %.3f, t_now_ = %.3f",
        rpm_cmd_, t_now_);

        rpm_cmd_ = rpm_init_ + ((t_now_ - t_start_) * acc_max_);
    }
    else if(t_now_ < (t_acc_ + t_cruise_ + t_start_))
    {
        // RCLCPP_INFO(this->get_logger(),
        // "Cruise phase: rpm_cmd_ = %.3f, t_now_ = %.3f",
        // rpm_cmd_, t_now_);

        double t_sum = t_acc_ + t_cruise_;

        RCLCPP_INFO(this->get_logger(),
        "Cruise phase - t_now: %.3f, t_acc+t_cruise: %.3f",
        t_now_, t_sum);

        rpm_cmd_ = vel_max_;
    }
    else if(t_now_ < (2*t_acc_ + t_cruise_ + t_start_))
    {
        RCLCPP_INFO(this->get_logger(),
        "Deceleration phase: rpm_cmd_ = %.3f, t_now_ = %.3f",
        rpm_cmd_, t_now_);

        rpm_cmd_ = vel_max_ - (t_now_ - t_cruise_ - t_acc_ - t_start_) * acc_max_;

        if(rpm_cmd_ < rpm_init_)
        {
            rpm_cmd_ = rpm_init_;
        }
    }
    else
    {
        rpm_cmd_ = rpm_init_;
    }

    cmd_msg_.header.stamp = this->now();
    cmd_msg_.cmd_raw = int16_t(rpm_cmd_*8191.0/9800.0);

    cmd_rpm_msg_.header.stamp = this->now();
    cmd_rpm_msg_.rpm = rpm_cmd_;

}