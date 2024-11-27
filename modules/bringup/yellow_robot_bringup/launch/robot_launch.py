import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('yellow_robot_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    ld = LaunchDescription()
    
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
    
    yellow_rviz = LaunchConfiguration('yellow_rviz')
    declare_yellow_rviz_arg = DeclareLaunchArgument(
        'yellow_rviz',
        default_value='True',
        description='Whether to start the rviz')
    
    rviz_yellow_config_file = LaunchConfiguration('rviz_yellow_config_file')
    declare_rviz_yellow_config_file = DeclareLaunchArgument(
        'rviz_yellow_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'default.rviz'),
        description='Full path to the RVIZ config file to use')

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_yellow_config_file],
        output='screen',
        condition=IfCondition(yellow_rviz),
        remappings=[
            ('/goal_pose', '/yellow_robot/goal_pose'),
            ('/tf', '/yellow_robot/tf'), ('/tf_static', '/yellow_robot/tf_static'),
        ],
        )

    ld.add_action(declare_yellow_rviz_arg)
    ld.add_action(declare_rviz_yellow_config_file)
    ld.add_action(rviz_cmd)

    return ld