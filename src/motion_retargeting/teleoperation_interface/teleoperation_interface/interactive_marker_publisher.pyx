# -----------------------------------------------------------
# File: interactive_marker_publisher.py
# Author: Hari Prasanth, Ng Yung Chuen
# Date Created: 30 January 2026

import os
import yaml
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from typing import Dict, List, Any
from geometry_msgs.msg import Pose, PoseArray
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (InteractiveMarker, InteractiveMarkerControl,
                                    InteractiveMarkerFeedback, Marker)


class HandMarkerNode(Node):
    """
    Creates and manages 6-DOF interactive markers in RViz for controlling robot hands.

    This node sets up two interactive markers, one for each hand. When a marker
    is moved, it publishes the poses of both hands to a PoseArray topic, which
    can be consumed by an IK solver.
    """

    def __init__(self):
        super().__init__('hand_interactive_marker_node')

        self.load_ROS2_param()

        # Check if robot_type loaded is valid
        if self.robot_type == "unknown_robot":
            raise ValueError("robot_type launch argument is not defined.")
        else:
            # Initialize the config loader.
            # Default config location in sde_robot_config
            try:
                package_share = self.get_sde_robot_config_package_dir()
                self.config_file_path = os.path.join(package_share, 'config', self.robot_type, 'data_collection', self.config_file_name)
            except Exception:
                raise FileNotFoundError("Could not find default config file. Please check config_file_path.")

        self.config = self.load_config()
        self.validate_config()
        self.setup_node_params()
        self.setup_communications()
        self.setup_markers()

        self.get_logger().info("Interactive Marker Node is ready. Look for markers in RViz.")

    def load_ROS2_param(self):
        """Load all input ROS2 parameters."""
        self.declare_parameter(
            'robot_type',
            '',
            ParameterDescriptor(
                description='Name of the robot'
            )
        )

        self.declare_parameter(
            'teleop_config_file',
            '',
            ParameterDescriptor(
                description='Name of the teleoperation interface YAML config file'
            )
        )

        try:
            self.robot_type = self.get_parameter('robot_type').value
            self.config_file_name = self.get_parameter('teleop_config_file').value
        except Exception as e:
            raise ValueError(f"Please check that your input arguments are correct: {e}")

    def load_config(self) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            with open(self.config_file_path, 'r') as file:
                config = yaml.safe_load(file)
                print(f"Successfully loaded config from {self.config_file_path}")
                return config
        except FileNotFoundError:
            raise FileNotFoundError(f"Config file not found: {self.config_file_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML config: {e}")

    def validate_config(self):
        """Validate that required configuration sections exist."""
        required_sections = ['ros2']
        for section in required_sections:
            if section not in self.config:
                raise ValueError(f"Missing required config section: {section}")
    
    def get_sde_robot_config_package_dir(self):
        """Get share directory path to YAML file."""
        return get_package_share_directory('sde_robot_config')

    def setup_node_params(self):
        """Declares and sets default values for the node's parameters."""
        self.robot_root_frame = self.config['ros2']['frames']['robot_root']
        self.control_poses_topic = self.config['ros2']['topics']['target_poses']
        self.marker_server_name = self.config['ros2']['ee_interactive_marker']['marker_server_name']
        self.marker_scale = 0.25
        self.box_scale = 0.08

        # Initial positions and orientations [x, y, z, qx, qy, qz, qw]
        self.left_ee_initial_poses = self.config['ros2']['ee_interactive_marker']['initial_poses']['left_ee']
        self.right_ee_initial_poses = self.config['ros2']['ee_interactive_marker']['initial_poses']['right_ee']
        self.default_head_pose = self._create_pose_from_list([0.0, 0.0, -0.0, -0.0, 0.0, 0.0, 1.0])
        self.num_target_poses = self.config['ros2']['topics']['num_target_poses']
        self.type_target_poses = self.config['ros2']['topics']['type_target_poses']

        # Colors [r, g, b, a]
        self.left_ee_marker_color = [0.8, 0.2, 0.2, 0.8]
        self.right_ee_marker_color = [0.2, 0.8, 0.2, 0.8]

    def setup_communications(self):
        """Initializes publishers and the interactive marker server."""
        self.pose_publisher = self.create_publisher(
                PoseArray,
                self.control_poses_topic,
                10
            )
        self.server = InteractiveMarkerServer(
                self,
                self.marker_server_name
            )

    def setup_markers(self):
        """Creates the initial interactive markers for both hands."""
        self.hand_poses = {}

        # Create markers using parameters
        left_pose_vals = self.left_ee_initial_poses
        left_color_vals = self.left_ee_marker_color
        initial_left_pose = self._create_pose_from_list(left_pose_vals)
        self.make_6dof_marker('left_hand', initial_left_pose, left_color_vals)

        right_pose_vals = self.right_ee_initial_poses
        right_color_vals = self.right_ee_marker_color
        initial_right_pose = self._create_pose_from_list(right_pose_vals)
        self.make_6dof_marker('right_hand', initial_right_pose, right_color_vals)

        self.server.applyChanges()

    def make_6dof_marker(self, name: str, initial_pose: Pose, color: list):
        """
        Creates a 6-DOF interactive marker with translation and rotation controls.

        Args:
            name: The unique name for the marker (e.g., 'left_hand').
            initial_pose: The starting pose of the marker.
            color: A list [r, g, b, a] for the marker's visual component.
        """
        self.hand_poses[name] = initial_pose

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.robot_root_frame
        int_marker.name = name
        int_marker.description = f"{name.replace('_', ' ').title()} Target"
        int_marker.pose = initial_pose
        int_marker.scale = self.marker_scale

        # Create a visual box marker for the center of the control
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_scale = self.box_scale
        box_marker.scale.x = box_scale
        box_marker.scale.y = box_scale
        box_marker.scale.z = box_scale
        box_marker.color.r = float(color[0])
        box_marker.color.g = float(color[1])
        box_marker.color.b = float(color[2])
        box_marker.color.a = float(color[3])

        # A non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        int_marker.controls.append(box_control)

        # Create controls for 6 degrees of freedom
        axes = [('x', [1.0, 0.0, 0.0]), ('y', [0.0, 1.0, 0.0]), ('z', [0.0, 0.0, 1.0])]
        for axis_name, orientation in axes:
            # Rotation control
            control = InteractiveMarkerControl()
            control.name = f"rotate_{axis_name}"
            control.orientation.w = 1.0
            control.orientation.x = orientation[0]
            control.orientation.y = orientation[1]
            control.orientation.z = orientation[2]
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)

            # Translation control
            control = InteractiveMarkerControl()
            control.name = f"move_{axis_name}"
            control.orientation.w = 1.0
            control.orientation.x = orientation[0]
            control.orientation.y = orientation[1]
            control.orientation.z = orientation[2]
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        """
        Callback function for marker interaction. Updates poses and publishes.

        Args:
            feedback: The feedback message from the interactive marker server.
        """
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.hand_poses[feedback.marker_name] = feedback.pose
            self.publish_poses()

    def publish_poses(self):
        """Assembles and publishes the current hand poses in a PoseArray."""
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.robot_root_frame

        # Ensure a consistent order (Left, then Right) for the IK solver
        if 'left_hand' in self.hand_poses and 'right_hand' in self.hand_poses:
            pose_array_msg.poses.append(self.hand_poses['left_hand'])
            pose_array_msg.poses.append(self.hand_poses['right_hand'])
            if 'head' in self.type_target_poses:
                pose_array_msg.poses.append(self.default_head_pose)
            if len(pose_array_msg.poses) == self.num_target_poses:
                self.pose_publisher.publish(pose_array_msg)
            else:
                self.get_logger().info("Interactive Marker Node unable to publish target as message do not contain corresponding number of predefined targets. Please check your config.")


    @staticmethod
    def _create_pose_from_list(values: list) -> Pose:
        """Helper function to create a Pose object from a list of 7 values."""
        pose = Pose()
        pose.position.x = values[0]
        pose.position.y = values[1]
        pose.position.z = values[2]
        pose.orientation.x = values[3]
        pose.orientation.y = values[4]
        pose.orientation.z = values[5]
        pose.orientation.w = values[6]
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = HandMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Interactive Marker Node...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
