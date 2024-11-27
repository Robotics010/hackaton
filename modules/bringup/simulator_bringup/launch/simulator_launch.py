import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            RegisterEventHandler, EmitEvent,
                            ExecuteProcess,
                            )
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_dir = get_package_share_directory('simulator_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ld = LaunchDescription()

    world = LaunchConfiguration('world')
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'eurobot_2025.model'),
        description='Full path to world model file to load')

    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -s ', world,
                                      # ' -v 4 ',
                                      ]}.items(),
    )

    ld.add_action(declare_world_arg)
    ld.add_action(gazebo_server_launch)

    headless = LaunchConfiguration('headless')
    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to start simulator without visualization')

    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        condition=UnlessCondition(headless),
        launch_arguments={'gz_args': ['-g ']}.items(),
    )

    ld.add_action(declare_headless_arg)
    ld.add_action(gazebo_client_launch)
    
    
    blue_robot = LaunchConfiguration('blue_robot')
    declare_blue_robot_arg = DeclareLaunchArgument(
        'blue_robot',
        default_value='True',
        description='Whether to start the blue robot software')
    ld.add_action(declare_blue_robot_arg)
    
    blue_robot_name = LaunchConfiguration('blue_robot_name')
    declare_blue_robot_name_arg = DeclareLaunchArgument(
        'blue_robot_name',
        default_value='blue',
        description='name of the robot')
    
    blue_robot_sdf = LaunchConfiguration('blue_robot_sdf')
    declare_blue_robot_sdf_arg = DeclareLaunchArgument(
        'blue_robot_sdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'blue.urdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    blue_pose_x = LaunchConfiguration('blue_pose_x')
    declare_blue_pose_x = DeclareLaunchArgument(
        'blue_pose_x',
        default_value='0.275',
        description='x position of robot')
    
    blue_pose_y = LaunchConfiguration('blue_pose_y')
    declare_blue_pose_y = DeclareLaunchArgument(
        'blue_pose_y',
        default_value='-0.75',
        description='y position of robot')
    
    blue_pose_yaw = LaunchConfiguration('blue_pose_yaw')
    declare_blue_pose_yaw = DeclareLaunchArgument(
        'blue_pose_yaw',
        default_value='1.57',
        description='yaw orientation of robot')
    
    spawn_blue_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_robot_launch.py')),
        launch_arguments={
            'robot_name': blue_robot_name,
            'robot_sdf': blue_robot_sdf,
            'pose_x': blue_pose_x,
            'pose_y': blue_pose_y,
            'pose_yaw': blue_pose_yaw,
        }.items(),
        condition=IfCondition(blue_robot),
    )

    ld.add_action(declare_blue_robot_name_arg)
    ld.add_action(declare_blue_robot_sdf_arg)
    ld.add_action(declare_blue_pose_x)
    ld.add_action(declare_blue_pose_y)
    ld.add_action(declare_blue_pose_yaw)
    ld.add_action(spawn_blue_robot_launch)

    yellow_robot = LaunchConfiguration('yellow_robot')
    declare_yellow_robot_arg = DeclareLaunchArgument(
        'yellow_robot',
        default_value='True',
        description='Whether to start the yellow robot software')
    ld.add_action(declare_yellow_robot_arg)

    yellow_robot_name = LaunchConfiguration('yellow_robot_name')
    declare_yellow_robot_name_arg = DeclareLaunchArgument(
        'yellow_robot_name',
        default_value='yellow',
        description='name of the robot')
    
    yellow_robot_sdf = LaunchConfiguration('yellow_robot_sdf')
    declare_yellow_robot_sdf_arg = DeclareLaunchArgument(
        'yellow_robot_sdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'yellow.urdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    yellow_pose_x = LaunchConfiguration('yellow_pose_x')
    declare_yellow_pose_x = DeclareLaunchArgument(
        'yellow_pose_x',
        default_value='-0.275',
        description='x position of robot')
    
    yellow_pose_y = LaunchConfiguration('yellow_pose_y')
    declare_yellow_pose_y = DeclareLaunchArgument(
        'yellow_pose_y',
        default_value='-0.75',
        description='y position of robot')
    
    yellow_pose_yaw = LaunchConfiguration('yellow_pose_yaw')
    declare_yellow_pose_yaw = DeclareLaunchArgument(
        'yellow_pose_yaw',
        default_value='1.57',
        description='yaw orientation of robot')
    
    spawn_yellow_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_robot_launch.py')),
        launch_arguments={
            'robot_name': yellow_robot_name,
            'robot_sdf': yellow_robot_sdf,
            'pose_x': yellow_pose_x,
            'pose_y': yellow_pose_y,
            'pose_yaw': yellow_pose_yaw,
        }.items(),
        condition=IfCondition(yellow_robot),
    )

    ld.add_action(declare_yellow_robot_name_arg)
    ld.add_action(declare_yellow_robot_sdf_arg)
    ld.add_action(declare_yellow_pose_x)
    ld.add_action(declare_yellow_pose_y)
    ld.add_action(declare_yellow_pose_yaw)
    ld.add_action(spawn_yellow_robot_launch)

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    ld.add_action(declare_use_sim_time_arg)

    shutdown_time = LaunchConfiguration('shutdown_time')
    declare_shutdown_time = DeclareLaunchArgument(
        'shutdown_time',
        default_value='100.0',
        description='Time in seconds before the whole simulation is ended')

    shutdown_timer_node = Node(
        package='timer',
        executable='timer_node',
        name='shutdown_timer',
        parameters=[
            {'use_sim_time': use_sim_time},
            {"stop_time": shutdown_time},
        ],
    )

    shutdown_on_timer = RegisterEventHandler(
        OnProcessExit(
            target_action=shutdown_timer_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Shutdown timer ticked'))],
        )
    )

    ld.add_action(declare_shutdown_time)
    ld.add_action(shutdown_timer_node)
    ld.add_action(shutdown_on_timer)

    return ld
