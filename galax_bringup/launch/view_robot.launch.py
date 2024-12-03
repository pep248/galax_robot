from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from xacro import process_file

def generate_launch_description():
    # Define the path to your xacro file
    package_name = 'galax_bringup'
    
    xacro_file_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'galax.urdf.xacro'
    )

    # Process the xacro file to generate URDF content
    urdf_file = process_file(xacro_file_path).toxml()

    return LaunchDescription([
            #static_transform_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
        ),
            
        # Load and visualize the laser's URDF in RViz
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}],
        ),

        # Launch RViz with a pre-configured config file if available
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory(package_name),
                'rviz',  # Folder containing RViz config (optional)
                'view_robot.rviz')]  # Optional pre-saved RViz config file
        ),

        # Joint State Publisher GUI to add sliders for joint movement
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
    ])
