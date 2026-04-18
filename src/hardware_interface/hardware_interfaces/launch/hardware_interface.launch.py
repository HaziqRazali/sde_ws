from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    cmd_type_arg = DeclareLaunchArgument(
        'cmd_type',
        default_value='1',  # CMD_TYPE_SERIAL
        description='Command type: 0=PARALLEL, 1=SERIAL'
    )

    client_ip_arg = DeclareLaunchArgument(
        'client_ip',
        default_value="127.0.0.1",
        description='Client IP address'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_name',
        default_value="booster_t1_hardware_interface.yaml",
        description='Configuration file name'
    )

    # Joint state to booster control converter node
    booster_t1_hardware_interface_node = Node(
        package='hardware_interfaces', 
        executable='booster_t1_hardware_interface',
        name='booster_t1_hardware_interface',
        output='screen',
        parameters=[{
            'cmd_type': LaunchConfiguration('cmd_type'),
            'client_ip': LaunchConfiguration('client_ip'),
            'config_name': LaunchConfiguration('config_name'),
        }],
        # remappings=[
        #     ('/joint_states', '/booster_t1/joint_states'),
        #     ('/joint_ctrl', '/booster_t1/low_cmd'),
        # ]
    )

    return LaunchDescription([
        cmd_type_arg,
        client_ip_arg,
        config_file_arg,
        booster_t1_hardware_interface_node,
    ])
