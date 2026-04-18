import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    robot_type_launch_arg = DeclareLaunchArgument('robot_type', 
                                                default_value='unknown_robot',
                                                description='Name of the robot to simulate'
                                            )
    robot_type = LaunchConfiguration('robot_type')

    sim_env_name_launch_arg = DeclareLaunchArgument('sim_env_name', 
                                                    default_value='default',
                                                    description='Name of the simulation environment to build'
                                                )
    sim_env_name = LaunchConfiguration('sim_env_name')

    enable_gui_launch_arg = DeclareLaunchArgument('enable_gui', 
                                                    default_value='true',
                                                    description='Set to true to enable GUI (default is true)'
                                                )
    enable_gui = LaunchConfiguration('enable_gui')

    num_ROS2_threads_launch_arg = DeclareLaunchArgument('num_ROS2_threads', 
                                                    default_value='4',
                                                    description='number of threads used for ROS2 executor'
                                                )
    num_ROS2_threads = LaunchConfiguration('num_ROS2_threads')

    enable_sim_tf_launch_arg = DeclareLaunchArgument('enable_sim_tf', 
                                                    default_value='true',
                                                    description='Set to true to enable simulation robot tf'
                                                )
    enable_sim_tf = LaunchConfiguration('enable_sim_tf')

    use_sim_time_launch_arg = DeclareLaunchArgument('use_sim_time', 
                                                    default_value='false',
                                                    description='Set to true to enable simulation time'
                                                )
    use_sim_time = LaunchConfiguration('use_sim_time')

    sde_simulation_pkg_share = get_package_share_directory('sde_simulation')
    rviz_config_file = os.path.join(sde_simulation_pkg_share, 'rviz/sim.rviz')

    # Simulation Node 
    sim_node = Node(
        package='sde_simulation',
        executable='sim_main_genesis',
        name='t1_sim_main_genesis',
        # namespace='',
        parameters=[{
            'robot_type': robot_type,
            'sim_env_name': sim_env_name,
            'enable_gui': enable_gui,
            'num_ROS2_threads': num_ROS2_threads,
            'enable_sim_tf': enable_sim_tf,
            'use_sim_time': use_sim_time,
            }],
        emulate_tty=True,             
        # prefix=['xterm -e gdb -ex run --args'], # this is used for C++
        output='screen'
    )

    ### To be launched by sim_node instead
    # Robot TF Publisher Node
    # sim_robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='sim_robot_state_publisher',
    #     output='screen',
    #     # parameters=[robot_description],
    #     remappings=[
    #         ('/joint_states', '/sim/joint_states'),
    #         ('/robot_description', ['/sim/robot_description']),
    #         ('/tf', '/sim/tf'),
    #         ('/tf_static', '/sim/tf_static'),
    #     ],
    #     condition=IfCondition(enable_sim_tf),
    # )

    # RViz2 Node 
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        remappings=[
            ('/robot_description', '/sim/robot_description'),
            ('/tf', '/sim/tf'),
            ('/tf_static', '/sim/tf_static'),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            }]
    )

    return LaunchDescription([
        robot_type_launch_arg,
        sim_env_name_launch_arg,
        enable_gui_launch_arg,
        num_ROS2_threads_launch_arg,
        enable_sim_tf_launch_arg,
        use_sim_time_launch_arg,
        sim_node,
        # sim_robot_state_publisher_node,
        rviz_node,
    ])
