from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    ld.add_action(declare_use_sim_time_arg)

    blue_robot = LaunchConfiguration('blue_robot')
    declare_blue_robot_arg = DeclareLaunchArgument(
        'blue_robot',
        default_value='True',
        description='Whether to start the blue robot software')

    blue_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('blue_robot_bringup'),
                'launch',
                'robot_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(blue_robot),
    )

    ld.add_action(declare_blue_robot_arg)
    ld.add_action(blue_robot_launch)
    
    yellow_robot = LaunchConfiguration('yellow_robot')
    declare_yellow_robot_arg = DeclareLaunchArgument(
        'yellow_robot',
        default_value='True',
        description='Whether to start the yellow robot software')

    yellow_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('yellow_robot_bringup'),
                'launch',
                'robot_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(yellow_robot),
    )

    ld.add_action(declare_yellow_robot_arg)
    ld.add_action(yellow_robot_launch)

    simulator = LaunchConfiguration('simulator')
    declare_simulator_arg = DeclareLaunchArgument(
        'simulator',
        default_value='True',
        description='Whether to start the simulator software')

    simulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('simulator_bringup'),
                'launch',
                'simulator_launch.py'
            ])
        ),
        condition=IfCondition(simulator),
    )

    ld.add_action(declare_simulator_arg)
    ld.add_action(simulator_launch)

    return ld
