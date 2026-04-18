# Copyright (c) 2026, Agency for Science, Technology and Research (A*STAR) All Rights Reserved.
# License TO-DO: Apache License, Version 2.0  
# Maintainer: Ng Yung Chuen, Email: ng_yung_chuen@a-star.edu.sg

import os
import yaml
import signal
import tempfile
import threading
import subprocess
import numpy as np
from typing import Dict, List, Any, Optional
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState, Image, Imu
from geometry_msgs.msg import Vector3, Quaternion


class ROS2Handler(Node): 
    """Handles all ROS2 object and interactions."""
    def __init__(self):
        super().__init__("ros2_handler_for_genesis_sim_node")
        self.get_logger().info("Initializing ROS2 handler for Genesis Simulation Node...")

        self.load_ROS2_param()

        if self.robot_type == "unknown_robot":
            raise ValueError("robot_type launch argument is not defined.")
        else:
            # Initialize the config loader.
            # Default config location in package
            try:
                package_share = self.get_sde_robot_config_package_dir()
                config_file_path = os.path.join(package_share, 'config', self.robot_type, 'sim_config.yaml')
            except Exception:
                raise FileNotFoundError("Could not find default config file. Please check config_file_path.")

        self.config_file_path = config_file_path
        self.config = self.load_config()
        # check for required config items
        self.validate_config()

        if self.num_ROS2_threads > 8:
            # self.get_logger().info('Limit set on number of ROS2 threads to maximum 8. Limiting to 8.')
            self.num_ROS2_threads = 8
        if self.num_ROS2_threads < 2:
            # self.get_logger().info('Limit set on number of ROS2 threads to minimum 2. Setting to 2.')
            self.num_ROS2_threads = 2
        # self.get_logger().info(f'Number of ROS2 threads set: {self.num_ROS2_threads}')
        self.cb_group = ReentrantCallbackGroup()

        # Initialise ROS2 topics and locks for comms topic
        self.subscriber = {}
        self.rgb_camera_publisher = {}
        self.depth_camera_publisher = {}
        self.render_img_freq = self.config['robot_model']['render_cam_freq']
        self.load_ROS2_topics()

        # load robot_description into ROS2 param
        with open(os.path.join(get_package_share_directory(self.config['robot_model']['description_package_name']), self.config['robot_model']['urdf_path']), 'r') as f:
            robot_description = f.read()
        # print(robot_description)
        params = {
            'robot_state_publisher': {
                'ros__parameters': {
                    'robot_description': robot_description
                }
            }
        }
        tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml')
        yaml.safe_dump(params, tmp)
        tmp.close()

        # self.declare_parameter('/sim/robot_description', robot_description)
        # self.get_logger().info('Loaded robot_description from URDF.')

        # launch robot_state_publisher
        if self.enable_sim_tf:
            self.robot_state_publisher_node_process = subprocess.Popen([
                'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args',
                '--params-file', tmp.name,
                '-r', '/joint_states:=/sim/joint_states',
                '-r', '/robot_description:=/sim/robot_description',
                '-r', '/tf:=/sim/tf',
                '-r', '/tf_static:=/sim/tf_static',
            ],
                preexec_fn=os.setsid   # ensure we can kill the whole process group
            )
            self.get_logger().info('robot_state_publisher process initialized.')
        else:
            self.robot_state_publisher_node_process = None

        self.get_logger().info('ROS2 Handler node initialized. Waiting to receive messages.')


    def load_ROS2_param(self):
        """Load all input ROS2 parameters."""
        self.declare_parameter(
            'robot_type',
            '',
            ParameterDescriptor(
                description='Type of robot to simulate'
            )
        )
        
        self.declare_parameter(
            'sim_env_name',
            '',
            ParameterDescriptor(
                description='Selected environment to generate'
            )
        )

        self.declare_parameter(
            'enable_gui',
            True,
            ParameterDescriptor(
                description='Set to true to visualize simulation'
            )
        )

        self.declare_parameter(
            'num_ROS2_threads',
            4,
            ParameterDescriptor(
                description='Selected environment to generate'
            )
        )
        
        self.declare_parameter(
            'enable_sim_tf',
            True,
            ParameterDescriptor(
                description='Set to true to enable simulation robot tf'
            )
        )

        try:
            self.robot_type = self.get_parameter('robot_type').value
            self.sim_env_name = self.get_parameter('sim_env_name').value
            self.enable_gui = self.get_parameter('enable_gui').value
            self.num_ROS2_threads = self.get_parameter('num_ROS2_threads').value
            self.enable_sim_tf = self.get_parameter('enable_sim_tf').value
        except Exception as e:
            raise ValueError(f"Please check that your input arguments are correct: {e}")

    def joint_states_callback(self, msg: JointState):
        """Receive joint states to control humanoid robot."""
        with self.subscriber['joint_states_callback']['lock']:
            self.joint_state_name = msg.name
            self.joint_state_cmd = msg.position
        try:
            assert len(self.joint_state_name) == len(self.joint_state_cmd)
        except Exception as e:
            self.get_logger().info(f"Joint commands len error: {e}")

    def publish_joint_states(self):
        """Publish latest joint states of humanoid robot."""
        with self.jstate_pub_lock:
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_publisher.publish(self.joint_state_msg)

    def publish_imu(self):
        """Publish latest imu data of humanoid robot."""
        with self.imu_pub_lock:
            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_publisher.publish(self.imu_msg)

    def publish_rgb_camera(self):
        """Publish latest rgb camera_images of humanoid robot."""
        # with self.cam_pub_lock:
        # self.rgb_camera_publisher[self.config['ROS2Topics']['rgb_camera_pub']['timer_fn_name']]['msg'].header.stamp = self.get_clock().now().to_msg()
        self.rgb_camera_publisher['publish_rgb_camera']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.rgb_camera_publisher['publish_rgb_camera']['publisher'].publish(self.rgb_camera_publisher['publish_rgb_camera']['msg'])

    def publish_depth_camera(self):
        """Publish latest depth camera_images of humanoid robot."""
        # with self.cam_pub_lock:
        # self.depth_camera_publisher[self.config['ROS2Topics']['depth_camera_pub']['timer_fn_name']]['msg'].header.stamp = self.get_clock().now().to_msg()
        self.depth_camera_publisher['publish_depth_camera']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.depth_camera_publisher['publish_depth_camera']['publisher'].publish(self.depth_camera_publisher['publish_depth_camera']['msg'])

    # TO-DO: correct the keys once validated
    def publish_rgb_camera1(self):
        """Publish latest rgb camera_images of humanoid robot for camera id 1."""
        # with self.cam_pub_lock:
        self.rgb_camera_publisher['publish_rgb_camera1']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.rgb_camera_publisher['publish_rgb_camera1']['publisher'].publish(self.rgb_camera_publisher['publish_rgb_camera1']['msg'])

    def publish_depth_camera1(self):
        """Publish latest depth camera_images of humanoid robot for camera id 1."""
        # with self.cam_pub_lock:
        self.depth_camera_publisher['publish_depth_camera1']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.depth_camera_publisher['publish_depth_camera1']['publisher'].publish(self.rgb_camera_publisher['publish_depth_camera1']['msg'])

    def publish_rgb_camera2(self):
        """Publish latest rgb camera_images of humanoid robot for camera id 2."""
        # with self.cam_pub_lock:
        self.rgb_camera_publisher['publish_rgb_camera2']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.rgb_camera_publisher['publish_rgb_camera2']['publisher'].publish(self.rgb_camera_publisher['publish_rgb_camera2']['msg'])

    def publish_depth_camera2(self):
        """Publish latest depth camera_images of humanoid robot for camera id 2."""
        # with self.cam_pub_lock:
        self.depth_camera_publisher['publish_depth_camera2']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.depth_camera_publisher['publish_depth_camera2']['publisher'].publish(self.rgb_camera_publisher['publish_depth_camera2']['msg'])

    def publish_rgb_camera3(self):
        """Publish latest rgb camera_images of humanoid robot for camera id 3."""
        # with self.cam_pub_lock:
        self.rgb_camera_publisher['publish_rgb_camera3']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.rgb_camera_publisher['publish_rgb_camera3']['publisher'].publish(self.rgb_camera_publisher['publish_rgb_camera3']['msg'])

    def publish_depth_camera3(self):
        """Publish latest depth camera_images of humanoid robot for camera id 3."""
        # with self.cam_pub_lock:
        self.depth_camera_publisher['publish_depth_camera3']['msg'].header.stamp = self.get_clock().now().to_msg()
        self.depth_camera_publisher['publish_depth_camera3']['publisher'].publish(self.rgb_camera_publisher['publish_depth_camera3']['msg'])

    def update_publish_data(self, jpos=None, jvel=None, jtorque=None, imu=None, rgb=None, depth=None, pub_cam_fn_name=None):
        """Update corresponding data of humanoid robot for publishing."""
        if jpos is not None:
            with self.jstate_pub_lock:
                self.joint_state_msg.position = jpos.tolist()
                self.joint_state_msg.velocity = jvel.tolist()
                self.joint_state_msg.effort = jtorque.tolist()
        if imu is not None:
            with self.imu_pub_lock:
                self.imu_msg.angular_velocity.x = float(imu[1][0])
                self.imu_msg.angular_velocity.y = float(imu[1][1])
                self.imu_msg.angular_velocity.z = float(imu[1][2])
                self.imu_msg.linear_acceleration.x = float(imu[0][0])
                self.imu_msg.linear_acceleration.y = float(imu[0][1])
                self.imu_msg.linear_acceleration.z = float(imu[0][2])
        if rgb is not None:
            # len(rgb)) # height, len(rgb[0])) # width. len(rgb[0][0])) # no. of channels
            # with self.rgb_camera_publisher[pub_cam_fn_name]['lock']:
            self.rgb_camera_publisher[pub_cam_fn_name]['msg'].data = rgb.flatten().tolist()
        if depth is not None: 
            if self.depth_camera_publisher[pub_cam_fn_name]['is_normalise']:
                norm_depth = (depth / depth.max()) * 255
                final_depth = np.round(norm_depth).astype(np.uint8) # uint16 depth data
            else:
                # Clamp to valid range of uint16
                clipped_depth = np.clip(depth, self.depth_camera_publisher[pub_cam_fn_name]['clip_range'][0], self.depth_camera_publisher[pub_cam_fn_name]['clip_range'][1])
                # Convert meters → depth units
                final_depth = (clipped_depth / self.depth_camera_publisher[pub_cam_fn_name]['depth_scale']).astype(np.uint16) # # uint16 depth data
            self.depth_camera_publisher[pub_cam_fn_name]['msg'].data = final_depth.tobytes()

    def get_sde_robot_config_package_dir(self):
        """Get share directory path to YAML file."""
        return get_package_share_directory('sde_robot_config')

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
        required_sections = ['simulation', 'robot_model', 'ROS2Topics']
        for section in required_sections:
            if section not in self.config:
                raise ValueError(f"Missing required config section: {section}")
    
    def spin_thread(self):
        """Spin ROS2 thread to receive updated ROS2 message."""
        executor = MultiThreadedExecutor(num_threads=self.num_ROS2_threads)
        executor.add_node(self)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    def load_ROS2_topics(self):
        # Load subscribers and publishers from config
        for topic in self.config['ROS2Topics']:
            if self.config['ROS2Topics'][topic]['type'] == 'subscriber':
                self.subscriber[self.config['ROS2Topics'][topic]['type_callback_fn']] = {}
                self.subscriber[self.config['ROS2Topics'][topic]['type_callback_fn']]['subscriber'] = self.create_subscription(
                    eval(self.config['ROS2Topics'][topic]['message_type']), # eval type
                    self.config['ROS2Topics'][topic]['name'],
                    eval("self." + self.config['ROS2Topics'][topic]['type_callback_fn']), # eval path to import function
                    self.config['ROS2Topics'][topic]['qos_depth'],
                    callback_group=self.cb_group
                    )
                self.subscriber[self.config['ROS2Topics'][topic]['type_callback_fn']]['lock'] = threading.Lock()
            if self.config['ROS2Topics'][topic]['type'] == 'publisher':
                if "joint_state" in topic:
                    self.joint_state_publisher = self.create_publisher(
                        eval(self.config['ROS2Topics'][topic]['message_type']),
                        self.config['ROS2Topics'][topic]['name'],
                        self.config['ROS2Topics'][topic]['qos_depth']
                    )
                    self.jstate_pub_lock = threading.Lock()
                    self.controllable_joint_names = tuple(self.config['robot_model']['controllable_joint_names'])
                    self.joint_state_name = None
                    self.joint_state_cmd = None
                    self.joint_state_msg = JointState()
                    self.joint_state_msg.name = list(self.controllable_joint_names)
                    self.joint_state_msg.position = [0.0] * len(self.joint_state_msg.name)
                    self.joint_state_pub_timer = self.create_timer(1.0 / self.config['ROS2Topics']['joint_state_pub']['publish_frequency'], self.publish_joint_states, callback_group=self.cb_group)
                if "imu" in topic:
                    self.imu_publisher = self.create_publisher(
                        eval(self.config['ROS2Topics'][topic]['message_type']),
                        self.config['ROS2Topics'][topic]['name'],
                        self.config['ROS2Topics'][topic]['qos_depth']
                    )
                    self.imu_pub_lock = threading.Lock()
                    quaternion = Quaternion()
                    quaternion.x = 0.0
                    quaternion.y = 0.0
                    quaternion.z = 0.0
                    quaternion.w = 0.0
                    self.imu_msg = Imu()
                    self.imu_msg.orientation = quaternion
                    self.imu_msg.orientation_covariance = self.config['ROS2Topics']['imu_pub']['orientation_covariance']
                    self.imu_msg.angular_velocity = Vector3(x = 0.0, y = 0.0, z = 0.0)
                    self.imu_msg.angular_velocity_covariance = self.config['ROS2Topics']['imu_pub']['angular_velocity_covariance']
                    self.imu_msg.linear_acceleration = Vector3(x = 0.0, y = 0.0, z = 0.0)
                    self.imu_msg.linear_acceleration_covariance = self.config['ROS2Topics']['imu_pub']['linear_acceleration_covariance']
                    self.imu_pub_timer = self.create_timer(1.0 / self.config['ROS2Topics']['imu_pub']['publish_frequency'], self.publish_imu, callback_group=self.cb_group)
                if "rgb_camera" in topic:
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']] = {}
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['publisher'] = self.create_publisher(
                        eval(self.config['ROS2Topics'][topic]['message_type']),
                        self.config['ROS2Topics'][topic]['name'],
                        self.config['ROS2Topics'][topic]['qos_depth']
                        )
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['publish_frequency'] = self.config['ROS2Topics'][topic]['publish_frequency']
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['lock'] = threading.Lock()
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'] = Image()
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].height = self.config['ROS2Topics'][topic]['height']
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].width = self.config['ROS2Topics'][topic]['width']
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].encoding = self.config['ROS2Topics'][topic]['encoding']
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].is_bigendian = 0
                    if self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].encoding == 'rgb8':
                        self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].step = self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].width * 3
                    else:
                        self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].step = self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].width
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].data = np.zeros(self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].height * self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].step, dtype=np.uint8).tolist()
                    self.rgb_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['timer'] = self.create_timer(1.0 / self.config['ROS2Topics'][topic]['publish_frequency'], eval("self." + self.config['ROS2Topics'][topic]['timer_fn_name']), callback_group=self.cb_group)
                
                if "depth_camera" in topic:
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']] = {}
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['publisher'] = self.create_publisher(
                        eval(self.config['ROS2Topics'][topic]['message_type']),
                        self.config['ROS2Topics'][topic]['name'],
                        self.config['ROS2Topics'][topic]['qos_depth']
                        )
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['publish_frequency'] = self.config['ROS2Topics'][topic]['publish_frequency']
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['lock'] = threading.Lock()
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'] = Image()
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].height = self.config['ROS2Topics'][topic]['height']
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].width = self.config['ROS2Topics'][topic]['width']
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].encoding = self.config['ROS2Topics'][topic]['encoding']
                    if self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].encoding == '16UC1':
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].step = self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].width * 2
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['depth_scale'] = 0.001
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['clip_range'] = (0.0, 65.535)
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['is_normalise'] = False
                    else:
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].step = self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].width
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['depth_scale'] = 0.001
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['clip_range'] = (0.0, 65.535)
                        self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['is_normalise'] = True
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].data = np.zeros(self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].height * self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['msg'].step, dtype=np.uint8).tolist()
                    self.depth_camera_publisher[self.config['ROS2Topics'][topic]['timer_fn_name']]['timer'] = self.create_timer(1.0 / self.config['ROS2Topics'][topic]['publish_frequency'], eval("self." + self.config['ROS2Topics'][topic]['timer_fn_name']), callback_group=self.cb_group)

        print("Loaded ROS2 Topic config:", self.config['ROS2Topics'])
        # print(type(self.config['ROS2Topics']))

    def kill_subprocess(self):
        try:
            os.killpg(os.getpgid(self.robot_state_publisher_node_process.pid), signal.SIGINT)
            self.get_logger().info("Successfully killed robot state publisher process.")
        except:
            self.get_logger().info("Robot state publisher process not found or already killed.")

    ### TEMPLATE
    # class apa():
        #     def __init__(self, a):
        #         self.aa = a
        #     def add(self):
        #         self.aa += 1
        #     def add2(self):
        #         eval("self.add()")