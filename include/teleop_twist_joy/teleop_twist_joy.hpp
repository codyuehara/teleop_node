#ifndef TELEOP_TWIST_JOY_HPP
#define TELEOP_TWIST_JOY_HPP

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace teleop_twist_joy
{

class TeleopTwistJoy : public rclcpp::Node
{
public:
    TeleopTwistJoy();
    
private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    
    int linear_z_axis_; // up/down left stick -> thrust
    int angular_x_axis_; // up/down right stick -> pitch
    int angular_y_axis_; // left/right right stick -> roll
    int angular_z_axis_; // left/right left stick -> yaw
    int enable_button_;

    //scaling
    double linear_z_scale_;
    double angular_x_scale_;
    double angular_y_scale_;
    double angular_z_scale_;

};

} // namespace

#endif
