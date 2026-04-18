#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ros2bag_server.srv import CollectRosbag
import subprocess
import os
import signal
import yaml
import datetime
import time
from typing import Dict, Any
from ament_index_python.packages import get_package_share_directory

def load_config(config_file_path) -> Dict[str, Any]:
    """Load configuration from YAML file."""
    try:
        with open(config_file_path, 'r') as file:
            config = yaml.safe_load(file)
            print(f"Successfully loaded config from {config_file_path}")
            return config
    except FileNotFoundError:
        raise FileNotFoundError(f"Config file not found: {config_file_path}")
    except yaml.YAMLError as e:
        raise ValueError(f"Error parsing YAML config: {e}")

class BagRecorderService(Node):
    def __init__(self):
        """
        Args:
            config_file_path: Path to YAML config file. If None, uses default location.
            output_dir_path: Path to directory to store ROS2 bag files.
        """
        super().__init__('bag_recorder_service')

        # Service
        self.srv = self.create_service(
            CollectRosbag,
            'collect_rosbag',
            self.handle_bag_service
        )

        # Declare ROS2 parameters in script
        self.declare_parameter(
            'robot_type',
            '',
            ParameterDescriptor(
                description='Type of robot or robot name'
            )
        )
        robot_type = self.get_parameter('robot_type').value
        if robot_type == "unknown_robot":
            raise ValueError("Please input robot type in launch config.")
        

        self.declare_parameter(
            'is_run_on_robot',
            False,
            ParameterDescriptor(
                description='Set to "true" if collecting data within robot instead, pointing to local ros2bag_server directory config file'
            )
        )
        is_run_on_robot = self.get_parameter('is_run_on_robot').value

        # Default config location in package
        self.declare_parameter(
            'config_file_name',
            '',
            ParameterDescriptor(
                description='Name of the YAML config file'
            )
        )
        config_file_name = self.get_parameter('config_file_name').value
        try:
            if is_run_on_robot:
                package_share = get_package_share_directory('ros2bag_server')
                config_file_path = os.path.join(package_share, 'config', config_file_name)
            else:
                package_share = self.get_sde_robot_config_package_dir()
                config_file_path = os.path.join(package_share, 'config', robot_type, 'data_collection', config_file_name)
        except Exception:
            raise FileNotFoundError("Could not find default config file. Please specify config_file_path.")
        self.get_logger().info(f"config_file_name at path: {config_file_path}")
        
        self.declare_parameter(
            'path_to_output_dir',
            '',
            ParameterDescriptor(
                description='Path to output directory for ROS2Bag Recorder Service Node'
            )
        )
        path_to_output_dir = self.get_parameter('path_to_output_dir').value
        if path_to_output_dir is None:
            raise ValueError("Path to output directory for ROS2 bag storage not specified.")

        self.declare_parameter(
            'output_dir',
            '',
            ParameterDescriptor(
                description='Bag file directory name to store bags'
            )
        )
        output_dir = self.get_parameter('output_dir').value
        if output_dir is None:
            raise ValueError("Output directory filename for ROS2 bag storage not specified.")

        self.declare_parameter(
            'is_mcap',
            False,
            ParameterDescriptor(
                description='Set to "true" to save in mcap format'
            )
        )
        self.is_mcap = self.get_parameter('is_mcap').value

        self.declare_parameter(
            'is_override_qos',
            False,
            ParameterDescriptor(
                description='Set to "true" to override qos type of topics'
            )
        )
        self.is_override_qos = self.get_parameter('is_override_qos').value

        package_share = get_package_share_directory('ros2bag_server')
        self.qos_profile_overrides_config_path = os.path.join(package_share, 'config', 'qos_overrides.yaml')
        if self.is_override_qos:
            self.get_logger().info(f"qos profile override defined at path: {self.qos_profile_overrides_config_path}")
        
        self.mcap_config_path = os.path.join(package_share, 'config', 'mcap_config.yaml')
        if self.is_mcap:
            self.get_logger().info(f"mcap config defined at path: {self.mcap_config_path}")

        self.output_dir_path = os.path.join(path_to_output_dir, output_dir)
        self.get_logger().info(f"Bag file storage path is: {self.output_dir_path}.")

        self.data_config = load_config(config_file_path)
        self.ros2topics_str = self.data_config['topics_to_collect']
        self.bagfile_name = self.data_config['bagfile_name']

        # Store the ros2 bag recording process
        self.bag_process = None
        self.episode_now = 0
        self.get_logger().info("Bag Recorder Service Ready.")

        self.delete_bag_process = None


    def handle_bag_service(self, request, response):
        command = request.command
        ep_label = request.episode

        if command == "start":
            return self.start_recording(response, ep_label)

        elif command == "stop":
            return self.stop_recording(response, ep_label, 0)
        elif command == "discard":
            return self.stop_recording(response, ep_label, 1)
        else:
            response.success = False
            response.message = "Unknown command. Use 'start' or 'stop'."
            return response

    # ------------------------ START RECORDING ------------------------

    def start_recording(self, response, ep_label):
        self.episode_now = ep_label
        if self.bag_process is not None:
            response.success = False
            response.message = "Recording already in progress."
            return response

        # Get the current date and time
        current_datetime = datetime.datetime.now()
        # Format the datetime object to display date and time up to the minute
        self.saved_bag_name = self.output_dir_path+self.bagfile_name+"_ep"+str(ep_label)+current_datetime.strftime("_%Y-%m-%d_%H-%M-%S")

        # ros2 bag command
        cmd = [
            "ros2", "bag", "record",
            "-o", self.saved_bag_name
        ]

        if self.is_override_qos:
            cmd.append("--qos-profile-overrides-path")
            cmd.append(self.qos_profile_overrides_config_path)

        if self.is_mcap:
            cmd.append("-s")
            cmd.append("mcap")
            cmd.append("--storage-config-file")
            cmd.append(self.mcap_config_path)

        for topic in self.ros2topics_str:
            cmd.append(topic)

        try:
            self.get_logger().info(f"Starting ros2 bag recording on episode {ep_label}.")

            # Start process
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid   # ensure we can kill the whole process group
            )

            response.success = True
            response.message = f"Recording started. Saving to {self.output_dir_path}."
            return response

        except Exception as e:
            response.success = False
            response.message = f"Failed to start recording: {e}"
            return response

    # ------------------------ STOP RECORDING ------------------------

    def stop_recording(self, response, ep_label, is_discard):
        if ep_label != self.episode_now:
            response.success = False
            response.message = f"Episode <{self.episode_now}> is recording now but requesting to stop recording of episode {ep_label}. Please check if collect_data service client is running correctly."
            return response
        if self.bag_process is None:
            response.success = False
            response.message = "No ros2 bag process running."
            return response

        self.get_logger().info(f"Stopping ros2 bag recording on episode {ep_label}.")

        try:
            # Kill the process group
            os.killpg(os.getpgid(self.bag_process.pid), signal.SIGINT)

            self.bag_process.wait()
            self.bag_process = None

            response.success = True
            response.message = "Recording stopped."

            if is_discard:
                time.sleep(1.0)

                # Start deleting process
                result = subprocess.run(
                        ["rm", "-rf", self.saved_bag_name],
                        check=False,
                )

                if result.returncode != 0:
                    self.get_logger().info("Bag deletion may have failed. Please double check manually.")
                else:
                    self.get_logger().info(f"Deleted ros2 bag for episode {ep_label}.")                

            return response

        except Exception as e:
            response.success = False
            response.message = f"Failed to stop episode {ep_label} recording: {e}"
            return response

    def get_sde_robot_config_package_dir(self):
        """Get share directory path to YAML file."""
        return get_package_share_directory('sde_robot_config')


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()