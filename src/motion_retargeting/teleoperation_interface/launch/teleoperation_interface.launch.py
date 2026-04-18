import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition # AndCondition not supported in ROS2 Humble
from launch_ros.actions import Node

def generate_launch_description():

    vr_type_launch_arg = DeclareLaunchArgument('vr_device', 
                                                default_value='disable',
                                                description='Name of the vr device',
                                                choices=['disable', 'metaquest3', 'htc_vive_pro2']
                                            )
    vr_device = LaunchConfiguration('vr_device')

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

    collect_data_arg = DeclareLaunchArgument('collect_data',
									        default_value='false',
									        description='Enable service to collect ROS2 bag data'
									    )
    collect_data = LaunchConfiguration('collect_data')

    teleop_config_file_arg = DeclareLaunchArgument('teleop_config_file',
                                            default_value='teleoperation_interface_config.yaml',
                                            description='Name of the teleoperation interface YAML config file'
                                        )
    teleop_config_file = LaunchConfiguration('teleop_config_file')

    task_config_file_arg = DeclareLaunchArgument('task_config_file',
                                            default_value='task_labelling_config.yaml',
                                            description='Name of the task labelling YAML config file'
                                        )
    task_config_file = LaunchConfiguration('task_config_file')

    enable_rviz_gui_arg = DeclareLaunchArgument('enable_rviz_gui', 
                                                default_value='false',
                                                description='Set to "true" to enable rviz and end-effector control from rviz'
                                            )
    enable_rviz_gui = LaunchConfiguration('enable_rviz_gui')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', 
                                                default_value='false',
                                                description='Set to "true" to enable rviz and end-effector control from rviz'
                                            )
    use_sim_time = LaunchConfiguration('use_sim_time')

    is_vr = PythonExpression([
	    "'false' if '",
	    vr_device,
	    "' == 'disable' else 'true'"
	])

    # VR Teleoperation Interface Node
    vr_teleoperation_interface_node = Node(
        package='teleoperation_interface',
        executable='vr_teleoperation_interface',
        name='vr_teleoperation_interface',
        parameters=[{
        	'vr_device': vr_device,
            'robot_type': robot_type,
            'sim_or_real': sim_or_real,
            'collect_data': collect_data,
            'teleop_config_file': teleop_config_file,
            'task_config_file': task_config_file,
            'use_sim_time': False,
            }],
        remappings=[
            ('/tf', ['/', sim_or_real, '/tf']),
            ('/tf_static', ['/', sim_or_real, '/tf_static']),
        ],
        output='screen',
        condition=IfCondition(is_vr),
    )

    # Interactive Marker Publisher Node
    interactive_marker_node = Node(
        package='teleoperation_interface',
        executable='interactive_marker_publisher',
        name='hand_interactive_marker_server',
        parameters=[{
            'robot_type': robot_type,
            'teleop_config_file': teleop_config_file,
            'use_sim_time': False,
            }],
        output='screen',
        # condition=IfCondition(enable_rviz_gui),
        condition=UnlessCondition(is_vr),
    )

    teleoperation_interface_pkg_share = get_package_share_directory('teleoperation_interface')
    rviz_config_file = os.path.join(teleoperation_interface_pkg_share, 'rviz/teleoperation_interface.rviz')

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
        parameters=[{
            'use_sim_time': use_sim_time,
            }]
    )
    
    return LaunchDescription([
        vr_type_launch_arg,
        robot_type_launch_arg,
        sim_or_real_arg,
        collect_data_arg,
        teleop_config_file_arg,
        task_config_file_arg,
        enable_rviz_gui_arg,
        vr_teleoperation_interface_node,
        interactive_marker_node,
        rviz_node,
    ])
