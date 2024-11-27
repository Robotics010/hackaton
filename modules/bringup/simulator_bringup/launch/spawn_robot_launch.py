import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('simulator_bringup')
    
    ld = LaunchDescription()
    
    robot_name = LaunchConfiguration('robot_name')
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='blue',
        description='name of the robot')
    
    robot_sdf = LaunchConfiguration('robot_sdf')
    declare_robot_sdf_arg = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'blue.urdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    pose_x = LaunchConfiguration('pose_x')
    declare_pose_x = DeclareLaunchArgument(
        'pose_x',
        default_value='0.275',
        description='x position of robot')
    
    pose_y = LaunchConfiguration('pose_y')
    declare_pose_y = DeclareLaunchArgument(
        'pose_y',
        default_value='-0.75',
        description='y position of robot')
    
    pose_z = LaunchConfiguration('pose_z')
    declare_pose_z = DeclareLaunchArgument(
        'pose_z',
        default_value='0.1981', # wheel_radius+(base_height/2) = 0.0381 + 0.32/2
        description='z position of robot')
    
    pose_yaw = LaunchConfiguration('pose_yaw')
    declare_pose_yaw = DeclareLaunchArgument(
        'pose_y',
        default_value='1.57',
        description='yaw position of robot')
    
    pose_roll = LaunchConfiguration('roll', default='0.00')
    pose_pitch = LaunchConfiguration('pitch', default='0.00')
    
    create_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-string', Command(['xacro', ' ', robot_sdf]),
            '-x', pose_x, '-y', pose_y, '-z', pose_z,
            '-R', pose_roll, '-P', pose_pitch, '-Y', pose_yaw,
            ]
    )
    
    ld.add_action(declare_pose_x)
    ld.add_action(declare_pose_y)
    ld.add_action(declare_pose_z)
    ld.add_action(declare_pose_yaw)
    ld.add_action(declare_robot_name_arg)
    ld.add_action(declare_robot_sdf_arg)
    ld.add_action(create_robot_cmd)
    
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": os.path.join(
                    bringup_dir, "params", "bridge.yaml"
                ),
            }
        ],
        output="screen",
    )

    ld.add_action(bridge_node)
    
    ign_converter = Node(
        package="ign_converter",
        executable="ign_converter_node",
    )

    ld.add_action(ign_converter)
    
    return ld
