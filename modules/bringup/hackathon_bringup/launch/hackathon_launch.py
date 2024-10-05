from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    robot = LaunchConfiguration('robot')
    declare_robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='True',
        description='Whether to start the robot software')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    drivers = LaunchConfiguration('drivers')
    declare_drivers_arg = DeclareLaunchArgument(
        'drivers',
        default_value='False',
        description='Whether to start the drivers module')
    
    control = LaunchConfiguration('control')
    declare_control_arg = DeclareLaunchArgument(
        'control',
        default_value='False',
        description='Whether to start the control module')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'robot_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'drivers': drivers,
            'control': control,
        }.items(),
        condition=IfCondition(robot),
    )

    ld.add_action(declare_robot_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_drivers_arg)
    ld.add_action(declare_control_arg)
    ld.add_action(robot_launch)

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
