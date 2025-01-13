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

    # robot_state_publisher and joint_state_publisher
    robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),
                    'launch',
                    'robot_state_publisher.launch.py')]),
                launch_arguments={'use_sim_time': 'true',
                                  'use_ros2_control': 'false'}.items()
    )


    # ros2 controller manager
    # ros2_controller will broadcast to the other controllers their configuration. It needs the robot_description parameter, which is passed via the /robot_description topic, published by the robot_state_publisher
    # ros2_controller_params_file = os.path.join(get_package_share_directory(package_name),
    #                                       'config',
    #                                       'controller_config.yaml')
    # ros2_controller = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[ros2_controller_params_file],
    #     remappings=[('/controller_manager/robot_description', '/robot_description')],
    # )
    # delayed_ros2_controller = TimerAction(
    #     period=3.0, 
    #     actions=[ros2_controller]
    # )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
                'world': os.path.join(get_package_share_directory(package_name), 'worlds', 'corridor.world'),
                'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
            }.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    # delayed_gazebo_spawner = TimerAction(
    #     period=5.0, 
    #     actions=[spawn_entity]
    # )
        
    # joint state broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    # delayed_joint_state_broadcaster = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=ros2_controller,
    #         on_start=[joint_state_broadcaster],
    #     )
    # )


    # diferential controller
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["differential_controller"], # receive the configuration from the ros2_control_node
    )
    # delayed_diff_drive_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessStart(
    #         target_action=ros2_controller,
    #         on_start=[diff_drive_controller_spawner],)
    # )


    # # Lidar
    # lidar_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('urg_node'),
    #         'launch',
    #         'urg_node.launch.py'
    #     )]),
    #     launch_arguments={
    #         'sensor_interface': 'ethernet'
    #     }.items()
    # )
    
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
        # delayed_diff_drive_controller_spawner,
        diff_drive_controller_spawner,
        gazebo,
        # delayed_gazebo_spawner,
        spawn_entity,
        # lidar_node,
        # rviz_node,
    ])
