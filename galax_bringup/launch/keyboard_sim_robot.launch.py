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
            'launch_sim_robot.launch.py'
        )]))

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    
    twist_multiplexor_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/differential_controller/cmd_vel_unstamped')]
        )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_node',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        robot_node,
        twist_multiplexor_node,
        teleop_node,
    ])
