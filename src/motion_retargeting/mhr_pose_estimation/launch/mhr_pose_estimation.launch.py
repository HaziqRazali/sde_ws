import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    robot_type_launch_arg = DeclareLaunchArgument('robot_type', 
                                                default_value='unknown_robot',
                                                description='Name of the robot'
                                            )
    robot_type = LaunchConfiguration('robot_type')

    sim_or_real_arg = DeclareLaunchArgument('sim_or_real',
                                            default_value='sim',
                                            description='Simulation or real robot'
                                        )
    sim_or_real = LaunchConfiguration('sim_or_real')

    config_file_arg = DeclareLaunchArgument('config_file',
                                            default_value='ik_solver_config.yaml',
                                            description='Name of the YAML config file'
                                        )
    config_file = LaunchConfiguration('config_file')

    enable_tf_arg = DeclareLaunchArgument('enable_tf', 
                                            default_value='false',
                                            description='Set to "true" to enable motion retargeted tf, and set robot_description ros2 param'
                                        )
    enable_tf = LaunchConfiguration('enable_tf')

    enable_rviz_gui_arg = DeclareLaunchArgument('enable_rviz_gui', 
                                                default_value='false',
                                                description='Set to "true" to enable rviz and end-effector control from rviz'
                                            )
    enable_rviz_gui = LaunchConfiguration('enable_rviz_gui')

    enable_static_world_tf_arg = DeclareLaunchArgument('enable_static_world_tf', 
                                                        default_value='false',
                                                        description='Set to "true" to enable static world tf. Please check that there is no odom source to avoid clash'
                                                    )
    enable_static_world_tf = LaunchConfiguration('enable_static_world_tf')

    robot_base_link_name_arg = DeclareLaunchArgument('robot_base_link_name', 
                                                        default_value='base_link',
                                                        description='Set robot base link name if different from "base_link"'
                                                )
    robot_base_link_name = LaunchConfiguration('robot_base_link_name')

    # Robot State Publisher Node (Deprecated)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='t1_robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        remappings=[
            ('/joint_states', ['/ik/joint_states']), # use to debug latency
            # ('/joint_states', ['/', sim_or_real, '/joint_states']),
            ('/robot_description', ['/', sim_or_real, '/robot_description']),
            ('/tf', ['/', sim_or_real, '/tf']),
            ('/tf_static', ['/', sim_or_real, '/tf_static']),
        ]
    )

    config_file_path = ''

    # IK Solver Node (Deprecated)
    demo_ik_solver_node = Node(
        package='mhr_pose_estimation',
        executable='demo_ik_collision_solver',
        name='t1_demo_ik_collision_solver',
        parameters=[{
            'config_file_path': config_file_path,
            'use_sim_time': False,
            }],
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/tf', ['/', sim_or_real, '/tf']),
            ('/tf_static', ['/', sim_or_real, '/tf_static']),
        ]
    )
    
    # Interactive Marker Publisher Node
    interactive_marker_node = Node(
        package='casadi_optimal_control_ik',
        executable='interactive_marker_publisher',
        name='hand_interactive_marker_server',
        parameters=[{
            'robot_type': robot_type,
            'config_file': config_file,
            'use_sim_time': False,
            }],
        output='screen',
        condition=IfCondition(enable_rviz_gui),
    )

    mhr_pose_estimation_pkg_share = get_package_share_directory('mhr_pose_estimation')
    rviz_config_file = os.path.join(mhr_pose_estimation_pkg_share, 'rviz/ik_demo.rviz')

    # RViz2 Node 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        remappings=[
            ('/robot_description', ['/', sim_or_real, '/robot_description']),
            ('/tf', ['/', sim_or_real, '/tf']),
            ('/tf_static', ['/', sim_or_real, '/tf_static']),
        ],
        condition=IfCondition(enable_rviz_gui),
    )
    
    # Publishes the transform from 'world' to the robot's root frame i.e. 'Trunk' or 'base_link'
    static_world_to_trunk_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_world_to_trunk_tf',
        arguments=['0', '0', '0.7',  # x, y, z
                   '0', '0', '0',      # roll, pitch, yaw
                   'world', robot_base_link_name], # reference_frame (parent), target_frame (child)
        remappings=[
            ('/tf', ['/', sim_or_real, '/tf']),
            ('/tf_static', ['/', sim_or_real, '/tf_static']),
        ],
        condition=IfCondition(enable_static_world_tf),
    )

    # pose estimator node
    pose_estimator_node = Node(
        package='mhr_pose_estimation',
        executable='pose_estimator',  # This must match your installed entry point
        name='pose_estimator_node',
        output='screen'
    )

    return LaunchDescription([
        robot_type_launch_arg,
        sim_or_real_arg,
        config_file_arg,
        enable_rviz_gui_arg,
        enable_tf_arg,
        enable_static_world_tf_arg,
        robot_state_publisher_node,
        demo_ik_solver_node,
        interactive_marker_node,
        rviz_node,
        static_world_to_trunk_tf_node,
        pose_estimator_node,
    ])
