import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ld = LaunchDescription()

    drivers = LaunchConfiguration('drivers')
    declare_drivers_arg = DeclareLaunchArgument(
        'drivers',
        default_value='True',
        description='Whether to start the drivers module')

    drivers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'drivers_launch.py')),
        condition=IfCondition(drivers),
    )

    ld.add_action(declare_drivers_arg)
    ld.add_action(drivers_launch)
    
    control = LaunchConfiguration('control')
    declare_control_arg = DeclareLaunchArgument(
        'control',
        default_value='True',
        description='Whether to start the control module')
    
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'control_launch.py')),
        condition=IfCondition(control),
    )
    
    ld.add_action(declare_control_arg)
    ld.add_action(control_launch)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    ld.add_action(declare_use_sim_time_arg)
    
    localization = LaunchConfiguration('localization')
    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='True',
        description='Whether to start the localization')
    
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(localization),
    )
    
    ld.add_action(declare_localization_cmd)
    ld.add_action(localization_launch)
    
    navigation = LaunchConfiguration('navigation')
    declare_navigation_cmd = DeclareLaunchArgument(
        'navigation',
        default_value='True',
        description='Whether to start the navigation')
    
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(navigation),
    )
    
    ld.add_action(declare_navigation_cmd)
    ld.add_action(navigation_launch)
    
    rviz = LaunchConfiguration('rviz')
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Whether to start the rviz')
    
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'default.rviz'),
        description='Full path to the RVIZ config file to use')

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(rviz),
        )

    ld.add_action(declare_rviz_arg)
    ld.add_action(declare_rviz_config_file)
    ld.add_action(rviz_cmd)

    return ld