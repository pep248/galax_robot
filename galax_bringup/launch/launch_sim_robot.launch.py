import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
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

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'empty.world')


    ## Gazebo nodes
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r ' + world
        }.items(),
    )

    # Spawn
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description','-z','0.05'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/world/diff_drive_world/model/diff_drive/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/diff_drive/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        parameters=[
                {
                    'qos_overrides./model/diff_drive.subscriber.reliability': 'reliable',
                    'qos_overrides./model/diff_drive.subscriber.reliability': 'reliable'
                }
        ],
        output='screen',
        remappings=[
            ('/model/diff_drive/odometry', '/odom'),
            ('/model/diff_drive/pose', '/tf'),
            ('/world/diff_drive_world/model/diff_drive/joint_state', '/joint_states'),
        ]
    )
    
    
    
    


    # # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='ros_gz_sim', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot'],
    #                     output='screen')
    # # delayed_gazebo_spawner = TimerAction(
    # #     period=5.0, 
    # #     actions=[spawn_entity]
    # # )
        
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
        gz_sim,
        bridge,
        # delayed_gazebo_spawner,
        spawn_entity,
        # lidar_node,
        # rviz_node,
    ])
