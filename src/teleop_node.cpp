#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "teleop_twist_joy/teleop_twist_joy.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<teleop_twist_joy::TeleopTwistJoy>();
    rclcpp::spin(node);

    rclcpp::shutdown();

}
