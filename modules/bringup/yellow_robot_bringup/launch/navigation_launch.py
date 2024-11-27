import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('yellow_robot_bringup')
    
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    ld.add_action(declare_use_sim_time_arg)
    
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='yellow_robot',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'params/navigation.yaml'),
            {
                'use_sim_time': use_sim_time,
            }],
        remappings=[
            ('/tf', '/yellow_robot/tf'), ('/tf_static', '/yellow_robot/tf_static'),
            ],
        )

    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='yellow_robot',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'params/navigation.yaml'),
            {
                'use_sim_time': use_sim_time,
            }],
        remappings=[
            ('/yellow_robot/map', '/yellow_robot/localization/map'),
            ('/tf', '/yellow_robot/tf'), ('/tf_static', '/yellow_robot/tf_static'),
            ],
        )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        namespace='yellow_robot',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'params/navigation.yaml'),
            {
                'use_sim_time': use_sim_time,
            }],
        remappings=[
            ('/yellow_robot/cmd_vel', '/yellow_robot/control/cmd_vel'),
            ('/yellow_robot/odom', '/yellow_robot/localization/odometry'),
            ('/tf', '/yellow_robot/tf'), ('/tf_static', '/yellow_robot/tf_static'),
            ('/trajectories', '/yellow_robot/trajectories'),
            ],
        )

    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='yellow_robot',
        output='screen',
        parameters=[
            os.path.join(bringup_dir, 'params/navigation.yaml'),
            {
                'use_sim_time': use_sim_time,
            }],
        remappings=[
            ('/yellow_robot/cmd_vel', '/yellow_robot/control/cmd_vel'),
            ('/tf', '/yellow_robot/tf'), ('/tf_static', '/yellow_robot/tf_static'),
            ],
        )
    
    ld.add_action(bt_navigator_node)
    ld.add_action(planner_node)
    ld.add_action(controller_node)
    ld.add_action(behavior_node)
    
    lifecycle_nodes = [
        'bt_navigator',
        'planner_server',
        'controller_server',
        'behavior_server',
    ]
    
    autostart = LaunchConfiguration('autostart')
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the navigation stack')
    
    lifecycle_man_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='yellow_robot',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ld.add_action(declare_autostart_arg)
    ld.add_action(lifecycle_man_node)
    
    goal_selector_node = Node(
        package='goal_selector_example',
        executable='goal_selector_example',
        name='goal_selector',
        namespace='yellow_robot',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
            }],
        remappings=[
            ('~/output/goal_pose', '/yellow_robot/goal_pose'),
            ('~/input/goal_completed', '/yellow_robot/goal_completed'),
            ('~/output/catch', '/yellow_robot/control/catch'),
            ('~/output/release', '/yellow_robot/control/release'),
            ('/tf', '/yellow_robot/tf'), ('/tf_static', '/yellow_robot/tf_static'),
            ],
        )
    
    ld.add_action(goal_selector_node)

    return ld
