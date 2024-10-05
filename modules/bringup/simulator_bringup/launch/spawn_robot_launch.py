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
        default_value='big_robotic',
        description='name of the robot')
    
    robot_sdf = LaunchConfiguration('robot_sdf')
    declare_robot_sdf_arg = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'big_robotic.urdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    pose = {'x': LaunchConfiguration('x_pose', default='1.275'),
            'y': LaunchConfiguration('y_pose', default='-0.775'),
            'z': LaunchConfiguration('z_pose', default='0.50'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='3.14')}
    
    create_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-string', Command(['xacro', ' ', robot_sdf]),
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )
    
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
