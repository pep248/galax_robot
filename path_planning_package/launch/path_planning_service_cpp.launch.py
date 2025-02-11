# Import the necessary launch libraries
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the service server node
        Node(
            package='path_planning_package',
            executable='path_planning_server',
            output='screen'
        ),

    ])
