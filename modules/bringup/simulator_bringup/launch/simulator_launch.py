import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_dir = get_package_share_directory('simulator_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ld = LaunchDescription()

    world = LaunchConfiguration('world')
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'world_only.model'),
        description='Full path to world model file to load')

    gazebo_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -s ', world]}.items(),
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

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'spawn_robot_launch.py')),
    )

    ld.add_action(spawn_robot_launch)

    return ld
