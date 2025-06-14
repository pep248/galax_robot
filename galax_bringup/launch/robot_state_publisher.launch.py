import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import launch_ros
import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    package_name = 'galax_description'
    
    # Process the URDF file
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'galax_robot.urdf.xacro'
    )

    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file,
                                    ' sim_mode:=', use_sim_time])
    
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # parameters=[{'robot_description': robot_description_config}]
        parameters=[{'robot_description': launch_ros.parameter_descriptions.ParameterValue(robot_description_config, value_type=str) }]
    )
        
    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        # joint_state_pub,
        node_robot_state_publisher,
        
    ])
