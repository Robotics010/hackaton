import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')

    ld = LaunchDescription()
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    ld.add_action(declare_use_sim_time_arg)
    
    map_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'params/localization.yaml'),
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': os.path.join(bringup_dir, 'maps/eurobot_world.yaml'),
            }
        ],
        remappings=[
            ('/map', '/robot/localization/map'),
        ],
    )

    ld.add_action(map_node)
    
    costmap_filter_info_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(bringup_dir, 'params/localization.yaml'),
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    ld.add_action(costmap_filter_info_node)
    
    lifecycle_nodes = [
        'map_server',
        'costmap_filter_info_server',
    ]
    
    autostart = LaunchConfiguration('autostart')
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the map stack')
    
    lifecycle_man_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])
    
    ld.add_action(declare_autostart_arg)
    ld.add_action(lifecycle_man_node)
    
    local_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='local_ekf_node',
       output='screen',
       parameters=[
           os.path.join(bringup_dir, 'params/localization.yaml'),
           {'use_sim_time': use_sim_time}],
       remappings=[
           ('/odometry/filtered', '/robot/localization/odometry'),
           ],
    )

    global_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='global_ekf_node',
       output='screen',
       parameters=[
           os.path.join(bringup_dir, 'params/localization.yaml'),
           {'use_sim_time': use_sim_time},
       ],
       remappings=[
           ('/odometry/filtered', '/robot/localization/global'),
       ],
    )
    
    ld.add_action(local_localization_node)
    ld.add_action(global_localization_node)
    
    return ld
