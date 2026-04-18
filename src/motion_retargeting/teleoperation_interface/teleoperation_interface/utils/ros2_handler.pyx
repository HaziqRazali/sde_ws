# Copyright (c) 2026, Agency for Science, Technology and Research (A*STAR) All Rights Reserved.
# License TO-DO: Apache License, Version 2.0  
# Maintainer: Ng Yung Chuen, Email: ng_yung_chuen@a-star.edu.sg
# File: vr_ik_teleop_node.py
# Description: An IK solver node that gets target poses from VR controllers
#              with an initial calibration step.

# TODO:
# 1. Clean up the mess 
# 3. The print statements are all over the place

import os
import sys 
import yaml 
import time
import threading
import numpy as np
import pinocchio as pin
from typing import Dict, Any
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from ros2bag_server.srv import CollectRosbag, SetTaskLabel, SetEpisode

from geometry_msgs.msg import Point, Pose, PoseArray
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, Int8, Int32, String, Int64MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import cv2
import tf2_ros


def pinSE3_to_Pose(pose):
    R = pose.rotation
    quat = pin.Quaternion(R)
    pose_data = Pose()
    pose_data.position.x = pose.translation[0]
    pose_data.position.y = pose.translation[1]
    pose_data.position.z = pose.translation[2]
    pose_data.orientation.x = quat.x
    pose_data.orientation.y = quat.y
    pose_data.orientation.z = quat.z
    pose_data.orientation.w = quat.w
    return pose_data

class ROS2Handler(Node): 
    """Handles all ROS2 object and interactions."""
    def __init__(self):
        super().__init__('vr_teleop_interface_node')
        self.get_logger().info('Initializing VR Teleop Interface Node...')

        self.load_ROS2_param()

        if self.robot_type == "unknown_robot":
            raise ValueError("robot_type launch argument is not defined.")
        else:
            # Initialize the config loader.
            # Default config location in package
            try:
                package_share = self.get_sde_robot_config_package_dir()
                teleop_config_file_path = os.path.join(package_share, 'config', self.robot_type, 'data_collection', self.teleop_config_file)
                task_config_file_path = os.path.join(package_share, 'config', self.robot_type, 'data_collection', self.task_config_file)
            except Exception:
                raise FileNotFoundError("Could not find default config file. Please check config_file_path.")

        self.teleop_config = self.load_config(teleop_config_file_path)
        self.task_config = self.load_config(task_config_file_path)
        self.verbose = self.teleop_config['verbose']

        # variables associated with task_config
        self.task_labelling_list = self.task_config["task_instruction_lists"]
        self.task_labelling_list_len = len(self.task_config["task_instruction_lists"])
        self.task_labelling_id = 0
        self.task_label = self.task_labelling_list[self.task_labelling_id]
        self.episode = 0
        self.is_done = False

        # check for required config items
        self.validate_config(self.teleop_config)

        self.num_ROS2_threads = 4
        # self.get_logger().info(f'Number of ROS2 threads set: {self.num_ROS2_threads}')
        self.cb_group = ReentrantCallbackGroup()

        self.enable_vr_cam_stream = self.teleop_config['head_camera_stream'][self.sim_or_real]['enable']
        self.use_compressed_image = self.teleop_config['head_camera_stream'][self.sim_or_real]['compressed']
        self.rgb_img_encoding = ""
        self.enable_gripper = False
        self.enable_hand = False

        self.load_ROS2_topics()

        self.get_logger().info('ROS2 Handler node initialized. Waiting to receive messages.')

        self.logger = self.get_logger() # For wrapper uasge

    def load_ROS2_param(self):
        """Load all input ROS2 parameters."""
        self.declare_parameter(
            'vr_device',
            '',
            ParameterDescriptor(
                description='Type of vr device to connect to'
            )
        )

        self.declare_parameter(
            'robot_type',
            '',
            ParameterDescriptor(
                description='Type of robot to teleop'
            )
        )

        self.declare_parameter(
            'sim_or_real',
            '',
            ParameterDescriptor(
                description='Simulation or real robot'
            )
        )

        self.declare_parameter(
            'collect_data',
            True,
            ParameterDescriptor(
                description='Set to true to enable service to collect ROS2 bag data'
            )
        )

        self.declare_parameter(
            'teleop_config_file',
            '',
            ParameterDescriptor(
                description='Name of the teleoperation interface YAML config file'
            )
        )
        
        self.declare_parameter(
            'task_config_file',
            '',
            ParameterDescriptor(
                description='Name of the task labelling YAML config file'
            )
        )

        try:
            self.vr_device = self.get_parameter('vr_device').value
            self.robot_type = self.get_parameter('robot_type').value
            self.sim_or_real = self.get_parameter('sim_or_real').value
            self.collect_data = self.get_parameter('collect_data').value
            self.teleop_config_file = self.get_parameter('teleop_config_file').value
            self.task_config_file = self.get_parameter('task_config_file').value
        except Exception as e:
            raise ValueError(f"Please check that your input arguments are correct: {e}")

    def get_sde_robot_config_package_dir(self):
        """Get share directory path to YAML file."""
        return get_package_share_directory('sde_robot_config')

    def load_config(self, config_file_path) -> Dict[str, Any]:
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

    def validate_config(self, config):
        """Validate that required configuration sections exist."""          
        required_sections = ['head_camera_stream', 'ros2', 'end_effector']
        if self.vr_device != 'disable':
            required_sections.append(self.vr_device)
        for section in required_sections:
            if section not in config:
                raise ValueError(f"Missing required config section: {section}")
    
    def load_ROS2_topics(self):
        """Load all ROS2 subscriber, publisher and service functions."""
        # tf listener for feedback joint states
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)

        # end effector tf listener
        self.trunk_frame = self.teleop_config['ros2']['frames']['robot_root']
        self.left_hand_ee_tf = self.teleop_config['ros2']['frames']['left_hand_fname']
        self.right_hand_ee_tf = self.teleop_config['ros2']['frames']['right_hand_fname']
        self.ee_tf_timer = self.create_timer(1.0 / self.teleop_config['ros2']['frames']['ee_frame_listen_rate'], self.update_real_ee_tf)

        ### initialize subscribers
        # manual signal to trigger data collection
        self.collect_data_flag_sub = self.create_subscription(
            Int8, 
            '/manual/collect_data', 
            self.collect_data_flag_callback, 
            10,
            callback_group=self.cb_group
        )

        # initialize streaming of camera images into VR
        if self.enable_vr_cam_stream:
            qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10
            )
            if self.use_compressed_image: 
                self.head_camera_stream_sub = self.create_subscription(
                    CompressedImage, 
                    self.teleop_config['head_camera_stream'][self.sim_or_real]['topic_name'], 
                    self.head_camera_stream_callback, 
                    qos, # 10
                    callback_group=self.cb_group
                )
                if "compressed" not in self.teleop_config['head_camera_stream'][self.sim_or_real]['topic_name']:
                    self.get_logger().info("Please check that the VR config for compressed image is properly set.")
            else:
                self.head_camera_stream_sub = self.create_subscription(
                    Image, 
                    self.teleop_config['head_camera_stream'][self.sim_or_real]['topic_name'], 
                    self.head_camera_stream_callback, 
                    qos, # 10
                    callback_group=self.cb_group
                )
                if "compressed" in self.teleop_config['head_camera_stream'][self.sim_or_real]['topic_name']:
                    self.get_logger().info("Please check that the VR setting for image is set correctly for non-compressed image.")
            self.cam_stream_lock = threading.Lock()
            self.vr_cam_height = self.teleop_config['head_camera_stream'][self.sim_or_real]['height']
            self.vr_cam_width = self.teleop_config['head_camera_stream'][self.sim_or_real]['width']
            if self.teleop_config['head_camera_stream'][self.sim_or_real]['encoding'] == "rgb8" or self.teleop_config['head_camera_stream'][self.sim_or_real]['encoding'] == "bgr8":
                self.img_data = np.zeros((self.vr_cam_height, self.vr_cam_width, 3), dtype=np.uint8)
                self.rgb_img_encoding = self.teleop_config['head_camera_stream'][self.sim_or_real]['encoding']
            elif self.teleop_config['head_camera_stream'][self.sim_or_real]['encoding'] == "16UC1":
                self.img_data = np.zeros((self.vr_cam_height, self.vr_cam_width, 2), dtype=np.uint8)
            else: 
                self.get_logger().info(f"Encoding input in ROS2_handler_node.teleop_config: {self.teleop_config['head_camera_stream'][self.sim_or_real]['encoding']}, not found, please check if you have keyed in a valid encoding.")
                self.destroy_node()
                rclpy.shutdown()

        ### initialize publishers
        self.control_pose_pub = self.create_publisher(PoseArray,
                                        self.teleop_config['ros2']['topics']['target_poses'],
                                        10)

        self.marker_pub = self.create_publisher(MarkerArray,
                                        '/vr_debug_markers',   # topic name
                                        10)                    # default QoS depth
        
        self.episode_idx_pub = self.create_publisher(Int32,
                                        '/episode_index',      # topic name
                                        10)                    # default QoS depth

        self.task_label_pub = self.create_publisher(String,
                                        '/task_label',         # topic name
                                        10)                    # default QoS depth

        self.is_done_pub = self.create_publisher(Bool,
                                        '/is_done',            # topic name
                                        10)                    # default QoS depth

        if self.teleop_config['end_effector']['type'] == 'gripper':
            self.enable_gripper = True
            self.gripper_pub = self.create_publisher(Int64MultiArray, '/hand_states', 10)
            self.gripper_lock = threading.Lock()
            self.gripper_state_msg = Int64MultiArray()
            self.gripper_min_pos = self.teleop_config['end_effector']['min_pos']
            self.gripper_max_pos = self.teleop_config['end_effector']['max_pos']
            self.gripper_state_msg.data = [self.gripper_max_pos, self.gripper_max_pos] # open gripper (left, right)
            self.gripper_publish_timer = self.create_timer(1.0 / self.teleop_config['end_effector']['frequency'], self.publish_gripper_states, callback_group=self.cb_group)

        ### initialize services
        # Create a client for the RosbagCommand service
        if self.collect_data:
            self.rosbag_client = self.create_client(CollectRosbag, 'collect_rosbag')
            self.get_logger().info("Waiting for /collect_rosbag service...")
            self.rosbag_client.wait_for_service()
            self.get_logger().info("/collect_rosbag service available.")

        # service server for setting task label
        self.task_label_srv = self.create_service(
            SetTaskLabel,
            'set_task_label',
            self.set_task_label_callback
        )

        self.set_episode_srv = self.create_service(
            SetEpisode,
            'set_episode',
            self.set_episode_callback
        )

    ############## ROS2 SUBSCRIBER FUNCTIONS ##############
    def collect_data_flag_callback(self, msg):
        """Manual signal trigger to start data collection."""
        self.manual_collect_data = msg.data
        return

    def head_camera_stream_callback(self, msg): 
        """Image for streaming back into VR headset."""
        img = np.frombuffer(msg.data, dtype=np.uint8)
        if self.use_compressed_image: 
            with self.cam_stream_lock:
                image = cv2.imdecode(img, cv2.IMREAD_COLOR) # for compressed images
            self.img_data = image.copy() # for compressed images
        else:
            ### Raw image does not need decoding
            with self.cam_stream_lock:
                self.img_data = img.reshape(self.vr_cam_height, self.vr_cam_width, 3)
        return

    def update_real_ee_tf(self):
        """TF lookup to find position of end-effector w.r.t. base_link (feedback pose)."""
        try:
            self.left_ee_tf = self.tf_buffer.lookup_transform(
                    self.trunk_frame,           # source frame / axis to use
                    self.left_hand_ee_tf,       # position of target_frame to acquire
                    rclpy.time.Time())
            self.right_ee_tf = self.tf_buffer.lookup_transform(
                    self.trunk_frame,           # source frame / axis to use
                    self.right_hand_ee_tf,       # position of target_frame to acquire
                    rclpy.time.Time())
            if self.verbose:
                self.get_logger().info(f'Transform found from trunk to end_effector: '
                                       f'Translation (left): x={self.left_ee_tf.transform.translation.x:.2f}, y={self.left_ee_tf.transform.translation.y:.2f}, z={self.left_ee_tf.transform.translation.z:.2f}'
                                       f'\nTranslation (right): x={self.right_ee_tf.transform.translation.x:.2f}, y={self.right_ee_tf.transform.translation.y:.2f}, z={self.right_ee_tf.transform.translation.z:.2f}')
        except:
            self.get_logger().info(f'Transform not found from {self.trunk_frame} to left or {self.right_hand_ee_tf}.')

    ############## ROS2 SUBSCRIBER FUNCTIONS ##############

    ############## ROS2 PUBLISHER FUNCTIONS ##############
    def publish_debug_markers(self, tgt_L, tgt_R, vr_L, vr_R,
                               T_rb_vw_L, T_rb_vw_R):
        """Debug markers for target VR pose from VR device."""
        ma = MarkerArray()
        next_id = 0
        # for pose in [tgt_L, tgt_R, vr_L, vr_R, T_rb_vw_L, T_rb_vw_R]:
        for pose in [tgt_L, tgt_R]:
            ma.markers.extend(self._make_axes(pose, next_id))
            next_id += 3
        self.marker_pub.publish(ma)

    def _make_axes(self, pose: pin.SE3, base_id: int, axis_len=0.10):
        """Create 3 arrow markers (RGB) visualising an SE3 frame."""
        colors = [(1.0,0.0,0.0,1.0), (0.0,1.0,0.0,1.0), (0.0,0.0,1.0,1.0)]
        origin  = pose.translation
        R       = pose.rotation
        markers = []
        for i in range(3):
            m = Marker()
            m.header.frame_id = self.teleop_config['ros2']['frames']['robot_root']
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns   = "vr_debug"
            m.id   = base_id + i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = axis_len * 0.8   # shaft length
            m.scale.y = axis_len * 0.05  # shaft diameter
            m.scale.z = axis_len * 0.05
            m.color.r, m.color.g, m.color.b, m.color.a = colors[i]
            start = Point(x=origin[0], y=origin[1], z=origin[2])
            end_vec = origin + R[:, i] * axis_len
            end   = Point(x=end_vec[0], y=end_vec[1], z=end_vec[2])
            m.points = [start, end]
            markers.append(m)
        return markers

    def publish_target_poses(self, pose_array):
        """Publish current episode index."""
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.teleop_config['ros2']['frames']['robot_root']
        for pose in pose_array:
            pose_array_msg.poses.append(pose)
        self.control_pose_pub.publish(pose_array_msg)

    def publish_episode_idx(self, idx):
        """Publish current episode index."""
        msg = Int32()
        msg.data = idx
        self.episode_idx_pub.publish(msg)

    def publish_task_label(self, task):
        """Publish current task label."""
        msg = String()
        msg.data = task
        self.task_label_pub.publish(msg)

    def publish_is_done(self, is_done):
        """Publish episode completion status."""
        msg = Bool()
        msg.data = is_done
        self.is_done_pub.publish(msg)

    def publish_gripper_states(self):
        """Publish current gripper states, if end effector is gripper, not None."""
        with self.gripper_lock:
            self.gripper_pub.publish(self.gripper_state_msg)
    ############## ROS2 PUBLISHER FUNCTIONS ##############

    ############## ROS2 SERVICE FUNCTIONS ##############
    def send_collect_data_service(self, command: str, ep_label: int):
        """
        Send a request to the bag control service without blocking.
        `command` must be "start" or "stop".
        """
        req = CollectRosbag.Request()
        req.command = command
        req.episode = ep_label

        # Send async request
        future = self.rosbag_client.call_async(req)

        # Add callback when service replies
        future.add_done_callback(self.collect_rosbag_srv_res_callback)

        self.get_logger().info(f"Sent async request: command={command}, ep_label={ep_label}")

    def collect_rosbag_srv_res_callback(self, future):
        """Called when ros2bag service returns a result."""
        try:
            result = future.result()
            self.get_logger().info(
                f"[/collect_data] service response: success={result.success}, message='{result.message}'"
            )
            if not result.success:
                if 'start' in result.message:
                    self.start_episode = False
                    self.get_logger().info(f"Episode {self.episode} has failed to start. Please check if data collection server is up.")
                if 'stop' in result.message:
                    self.start_episode = True
                    self.get_logger().info(f"Episode {self.episode} has failed to stop. Please check if data collection server is up.")
                    self.episode = int(result.message.split("<")[1].split(">")[0])
                    # self.episode -= 1 # does not work when drifts too far
                    self.get_logger().info(f"Synced episode number to {self.episode} as referenced in collect_data service server. Please click A button to stop the episode and recording again, or reset the episode by clicking on X.")
        except Exception as e:
            self.get_logger().info(f"[/collect_data] service call failed: {e}")
    
    def set_task_label_callback(self, request, response):
        """manual service call to change and add tasks to list."""
        self.get_logger().info(f"Received request to set task label: '{request.task_label}'")
        self.task_label = request.task_label
        if self.task_label not in self.task_labelling_list:
            self.task_labelling_list.append(self.task_label)
            self.task_labelling_list_len = len(self.task_labelling_list)
        self.task_labelling_id = self.task_labelling_list.index(self.task_label)
        response.success = True
        return response

    def set_episode_callback(self, request, response):
        """manual service call to set and change current episode number."""
        self.get_logger().info(f"Received request to set episode number: '{request.episode_number}'")
        self.episode = request.episode_number
        response.success = True
        return response

    ############## ROS2 SERVICE FUNCTIONS ##############

    def spin_thread(self):
        """Spin ROS2 thread to receive updated ROS2 message."""
        executor = MultiThreadedExecutor(num_threads=self.num_ROS2_threads)
        executor.add_node(self)
        try:
            executor.spin()
        finally:
            executor.shutdown()