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

    # robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),
                    'launch',
                    'robot_state_publisher.launch.py')]),
                launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo
    gazebo_params_file = os.path.join(get_package_share_directory('amr-ros-config'),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world'),
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
            }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'galax_robot',],
                        output='screen')

    # joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # diferential controller
    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"], # receive the configuration from the ros2_control_node
    )
  
    # rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory(package_name),
            'rviz',  # Folder containing RViz config (optional)
            'robot_display.rviz')]  # Optional pre-saved RViz config file
    )


    # Launch them all!
    return LaunchDescription([
        robot_state_publisher,
        # delayed_ros2_controller,
        # delayed_joint_state_broadcaster,
        joint_state_broadcaster,
        # delayed_diff_drive_controller,
        diff_drive_controller,
        gazebo,
        # delayed_gazebo_spawner,
        spawn_entity,
        # rviz_node,
    ])
