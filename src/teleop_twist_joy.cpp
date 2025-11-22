#include "teleop_twist_joy/teleop_twist_joy.hpp"

using std::placeholders::_1;

namespace teleop_twist_joy
{

TeleopTwistJoy::TeleopTwistJoy()
: Node("teleop_twist_joy")
{

    linear_z_axis_ = this->declare_parameter<int>("axis_linear_z", 1);// left stick vertical
    angular_x_axis_ = this->declare_parameter<int>("axis_angular_x", 4); // right stick vertical
    angular_y_axis_ = this->declare_parameter<int>("axis_angular_y", 3); // right stick horizontal
    angular_z_axis_ = this->declare_parameter<int>("axis_angular_z", 0); // left stick horizontal

    enable_button_ = this->declare_parameter<int>("enable_button", 5); // RB button

    linear_z_scale_ = this->declare_parameter<double>("scale_linear_z", 1.0);
    angular_x_scale_ = this->declare_parameter<double>("scale_angular_x", 1.0);
    angular_y_scale_ = this->declare_parameter<double>("scale_angular_y", 1.0);
    angular_z_scale_ = this->declare_parameter<double>("scale_angular_z", 1.0);

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd", 10);

    joy_sub_= this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&TeleopTwistJoy::joyCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "6-DOF TeleopTwistJoy node started");
}

void TeleopTwistJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // check if enable button is pressed 
    if (enable_button_ >= 0 && enable_button_ < static_cast<int>(msg->buttons.size()))
    {
        if (msg->buttons[enable_button_] == 0) 
        {
            return;
        }
    }

    geometry_msgs::msg::Twist twist;

    if (linear_z_axis_ >= 0 && linear_z_axis_ < static_cast<int>(msg->axes.size()))
    {
        twist.linear.z = msg->axes[linear_z_axis_] * linear_z_scale_;
    }

          // --- Angular X (pitch) ---
      if (angular_x_axis_ >= 0 && angular_x_axis_ < static_cast<int>(msg->axes.size()))
      {
        twist.angular.x = msg->axes[angular_x_axis_] * angular_x_scale_;
      }

      // --- Angular Y (roll) ---
      if (angular_y_axis_ >= 0 && angular_y_axis_ < static_cast<int>(msg->axes.size()))
      {
        twist.angular.y = msg->axes[angular_y_axis_] * angular_y_scale_;
      }

      // --- Angular Z (yaw) ---
      if (angular_z_axis_ >= 0 && angular_z_axis_ < static_cast<int>(msg->axes.size()))
      {
        twist.angular.z = msg->axes[angular_z_axis_] * angular_z_scale_;
      }

      cmd_pub_->publish(twist);
}

} // end namespace
