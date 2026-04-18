import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,  EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    robot_type_launch_arg = DeclareLaunchArgument(
        'robot_type', 
        default_value='unknown_robot',
        description='Name of the robot to simulate'
    )
    robot_type = LaunchConfiguration('robot_type')

    config_file_arg = DeclareLaunchArgument(
        'config_name',
        default_value='data_collection_config.yaml',
        description='Name of the YAML config file'
    )
    config_file_name = LaunchConfiguration('config_name')

    path_to_output_dir_arg = DeclareLaunchArgument(
        'path_to_output_dir',
        default_value=['/home/', EnvironmentVariable('USER')],
        description='Path to directory to store saved bag files'
    )
    path_to_output_dir = LaunchConfiguration('path_to_output_dir')

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='bag/',
        description='Directory to store saved bag files'
    )
    output_dir = LaunchConfiguration('output_dir')

    is_run_on_robot_arg = DeclareLaunchArgument(
        'is_run_on_robot',
        default_value='false',
        description='Set to "true" if collecting data within robot instead, pointing to local ros2bag_server directory config file'
    )
    is_run_on_robot = LaunchConfiguration('is_run_on_robot')

    is_mcap_arg = DeclareLaunchArgument(
        'is_mcap',
        default_value='false',
        description='Set to "true" to save in mcap format'
    )
    is_mcap = LaunchConfiguration('is_mcap')

    is_override_qos_arg = DeclareLaunchArgument(
        'is_override_qos',
        default_value='false',
        description='Set to "true" to override qos type of topics'
    )
    is_override_qos = LaunchConfiguration('is_override_qos')

    # ros2bag server node
    ros2bag_server_node = Node(
        package='ros2bag_server',
        executable='ros2bag_server_node.py',
        name='ros2bag_server',
        output='screen',
        parameters=[{
            'robot_type': robot_type,
            'config_file_name': config_file_name,
            'path_to_output_dir': path_to_output_dir,
            'output_dir': output_dir,
            'is_run_on_robot': is_run_on_robot,
            'is_mcap': is_mcap,
            'is_override_qos': is_override_qos,
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        robot_type_launch_arg,
        config_file_arg,
        path_to_output_dir_arg,
        output_dir_arg,
        is_run_on_robot_arg,
        is_mcap_arg,
        is_override_qos_arg,
        config_file_arg,
        ros2bag_server_node,
    ])
