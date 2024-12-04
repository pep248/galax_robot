import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():
    package_name='galax_bringup'

    robot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),
            'launch',
            'launch_real_robot.launch.py'
        )]))

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_node',
        output='screen',
        prefix='xterm -e',
        remappings=[
            ('/cmd_vel', '/differential_controller/cmd_vel_unstamped'),
        ]
    )

    return LaunchDescription([
        robot_node,
        teleop_node,
    ])
