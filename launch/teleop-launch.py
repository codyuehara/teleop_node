import os

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    # Launch arguments
    joy_vel = launch.substitutions.LaunchConfiguration('joy_vel')
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        # Declare arguments
        launch.actions.DeclareLaunchArgument(
            'joy_vel', default_value='cmd',
            description='Topic to publish velocity commands'),
        launch.actions.DeclareLaunchArgument(
            'joy_config', default_value='ps4',
            description='Joystick config name'),
        launch.actions.DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='Joystick device path'),
        launch.actions.DeclareLaunchArgument(
            'config_filepath',
            default_value=[
                launch.substitutions.TextSubstitution(text=os.path.join(
                    get_package_share_directory('teleop_twist_joy'), 'config', '')),
                joy_config,
                launch.substitutions.TextSubstitution(text='.yaml')
            ],
            description='Path to joystick YAML configuration'),

        # Joy node
        launch_ros.actions.Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),

        # Teleop node
        launch_ros.actions.Node(
            package='teleop_node',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath],
            remappings=[('/cmd', joy_vel)],
        ),
    ])

