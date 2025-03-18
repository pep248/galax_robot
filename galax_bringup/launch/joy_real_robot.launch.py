import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'galax_bringup'

    # Include the robot launch file if necessary
    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),
            'launch',
            'launch_real_robot.launch.py'
        )])
    )

    # Declare launch arguments for teleop
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    publish_stamped_twist = LaunchConfiguration('publish_stamped_twist')
    config_filepath = LaunchConfiguration('config_filepath')

    return LaunchDescription([
        # Robot node launch (uncomment if needed)
        robot_node,

        # Declare launch arguments for teleop configuration
        DeclareLaunchArgument('joy_vel', default_value='/differential_controller/cmd_vel_unstamped'),
        DeclareLaunchArgument('joy_config', default_value='ps3'),
        DeclareLaunchArgument('joy_dev', default_value='0'),
        DeclareLaunchArgument('publish_stamped_twist', default_value='false'),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config, TextSubstitution(text='.config.yaml')]),

        # joy_node for handling joystick input
        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'device_id': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),

        # teleop_twist_joy_node for processing joystick commands
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[config_filepath, {'publish_stamped_twist': publish_stamped_twist}],
            remappings=[('/cmd_vel', LaunchConfiguration('joy_vel'))],
        ),
    ])
