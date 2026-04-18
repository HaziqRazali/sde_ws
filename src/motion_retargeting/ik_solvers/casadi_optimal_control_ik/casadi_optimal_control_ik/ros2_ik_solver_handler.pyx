# -----------------------------------------------------------
# File: ik_solver_node.py
# Description: Robot IK solver node that subscribes to a PoseArray topic.
#
# Author: Hari Prasanth, Ng Yung Chuen
# Date Created: 30 January 2026

import os
import yaml
import signal
import tempfile
import threading
import subprocess

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, TransformStamped
import tf2_ros
import numpy as np
import pinocchio as pin
from casadi_optimal_control_ik.ik_core import RobotArmIK, T_mat_to_euler_angles


class ROS2IKSolverHandler(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        self.get_logger().info('Initializing ROS2 Handler for Robot IK Solver Node...')

        self.load_ROS2_param()
        
        if self.robot_type == "unknown_robot":
            raise ValueError("robot_type launch argument is not defined.")
        else:
            # Initialize the config loader.
            # Default config location in sde_robot_config
            try:
                package_share = self.get_sde_robot_config_package_dir()
                config_file_path = os.path.join(package_share, 'config', self.robot_type, self.config_file_name)
            except Exception:
                raise FileNotFoundError("Could not find default config file. Please check config_file_path.")
        if self.sim_or_real not in ['sim','real']:
            raise ValueError("sim_or_real launch argument is neither 'sim' nor 'real'.")

        self.robot_arm_ik = RobotArmIK(config_file_path)
        self.ik_config = self.robot_arm_ik.config

        # load robot_description into ROS2 param
        with open(os.path.join(get_package_share_directory(self.ik_config.DESCRIPTION_PACKAGE_NAME), self.ik_config.URDF_PATH), 'r') as f:
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

        # launch robot_state_publisher
        if self.enable_tf:
            self.robot_state_publisher_node_process = subprocess.Popen([
                'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                '--ros-args',
                '-p', f'use_sim_time:={self.use_sim_time}', 
                '--params-file', tmp.name,
                '-r', f'/joint_states:=/ik/joint_states',
                '-r', f'/robot_description:=/{self.sim_or_real}/robot_description',
                '-r', f'/tf:=/{self.sim_or_real}/tf',
                '-r', f'/tf_static:=/{self.sim_or_real}/tf_static',
            ],
                preexec_fn=os.setsid   # ensure we can kill the whole process group
            )
            self.get_logger().info('robot_state_publisher process initialized.')
        else:
            self.robot_state_publisher_node_process = None

        # define number of threads to use for all ROS2 subscribers and publishers
        self.num_ROS2_threads = 4
        self.cb_group = ReentrantCallbackGroup()

        # Initialise ROS2 subscribers and publishers
        self.joint_pub = self.create_publisher(JointState, self.ik_config.JOINT_STATE_TOPIC, self.ik_config.QOS_DEPTH)
        self.target_sub = self.create_subscription(
            PoseArray, 
            self.ik_config.TARGET_POSE_TOPIC, 
            self.solve_ik_callback, 
            self.ik_config.QOS_DEPTH,
            callback_group=self.cb_group
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.lock = threading.Lock()
        
        self.full_joint_names = [name for name in self.robot_arm_ik.full_robot.model.names[1:]]
        reduced_joint_names = [name for name in self.robot_arm_ik.reduced_robot.model.names[1:]]
        self.reduced_to_full_map = [self.full_joint_names.index(name) for name in reduced_joint_names]

        # for idx, name in enumerate(self.full_joint_names):
        #     if name == "Waist_joint":
        #         self.full_joint_names[idx] = "Waist"

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.full_joint_names
        
        ### Initialize joint state position with reference configuration
        self.joint_state_msg.position = list(self.ik_config.REFERENCE_CONFIGURATION)
        
        self.timer = self.create_timer(1.0 / self.ik_config.PUBLISH_RATE_HZ, self.publish_joint_states, callback_group=self.cb_group)
        
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.ee_tf_timer = self.create_timer(1.0 / self.ik_config.EE_TF_SAMPLE_HZ, self.update_real_ee_tf, callback_group=self.cb_group)

        self.get_logger().info('Demo node initialized. Waiting for target poses on /control_poses_target.')

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
            'sim_or_real',
            '',
            ParameterDescriptor(
                description='Simulation or real robot'
            )

        )

        self.declare_parameter(
            'config_file',
            '',
            ParameterDescriptor(
                description='Name of the YAML config file'
            )
        )
        
        self.declare_parameter(
            'enable_tf',
            True,
            ParameterDescriptor(
                description='Set to "true" to enable motion retargeted tf, and set robot_description ros2 param'
            )
        )

        try:
            self.robot_type = self.get_parameter('robot_type').value
            self.sim_or_real = self.get_parameter('sim_or_real').value
            self.config_file_name = self.get_parameter('config_file').value
            self.enable_tf = self.get_parameter('enable_tf').value
            self.use_sim_time = self.get_parameter('use_sim_time').value
        except Exception as e:
            raise ValueError(f"Please check that your input arguments are correct: {e}")

    def publish_joint_states(self):
        with self.lock:
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.joint_state_msg)

    def pose_to_se3(self, pose):
            p = np.array([pose.position.x, pose.position.y, pose.position.z])
            q = pin.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            return pin.SE3(q.toRotationMatrix(), p)

    def solve_ik_callback(self, msg: PoseArray):
        try:
            if len(msg.poses) != self.ik_config.NUM_TARGET_POSES:
                self.get_logger().info(f"Received PoseArray with {len(msg.poses)} poses, but expected {self.ik_config.NUM_TARGET_POSES}. Ignoring.")
                return
        except:
            self.get_logger().info(f"Waiting to receive message at topic {self.ik_config.TARGET_POSE_TOPIC}.")

        L_tf_target = self.pose_to_se3(msg.poses[0])
        R_tf_target = self.pose_to_se3(msg.poses[1])
        if 'head' in self.ik_config.TYPE_TARGET_POSES:
            head_tf_target = self.pose_to_se3(msg.poses[2])
            euler_rad = T_mat_to_euler_angles(head_tf_target.homogeneous) # [y, p, r]
        
        try:
            arm_solution_q = self.robot_arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)
            with self.lock:
                for i, q_val in enumerate(arm_solution_q):
                    full_model_index = self.reduced_to_full_map[i]
                    self.joint_state_msg.position[full_model_index] = q_val

                # assign head orientation to joint state published
                    for head_joint_name in self.ik_config.HEAD_CONTROL_JOINTS:
                        if 'yaw' in head_joint_name:
                            self.joint_state_msg.position[self.joint_state_msg.name.index(head_joint_name)] = euler_rad[0]
                        elif 'pitch' in head_joint_name:
                            self.joint_state_msg.position[self.joint_state_msg.name.index(head_joint_name)] = euler_rad[1]
                        else:
                            break

            self.publish_target_transforms(msg)
        except Exception as e:
            self.get_logger().info(f"IK solver failed: {e}")

    def publish_target_transforms(self, msg: PoseArray):
        t_left = TransformStamped()
        t_left.header.stamp = self.get_clock().now().to_msg()
        t_left.header.frame_id = self.ik_config.ROBOT_ROOT_FRAME
        t_left.child_frame_id = self.ik_config.LEFT_TARGET_TF_FRAME
        t_left.transform.translation.x = msg.poses[0].position.x
        t_left.transform.translation.y = msg.poses[0].position.y
        t_left.transform.translation.z = msg.poses[0].position.z
        t_left.transform.rotation = msg.poses[0].orientation
        t_right = TransformStamped()
        t_right.header.stamp = self.get_clock().now().to_msg()
        t_right.header.frame_id = self.ik_config.ROBOT_ROOT_FRAME
        t_right.child_frame_id = self.ik_config.RIGHT_TARGET_TF_FRAME
        t_right.transform.translation.x = msg.poses[1].position.x
        t_right.transform.translation.y = msg.poses[1].position.y
        t_right.transform.translation.z = msg.poses[1].position.z
        t_right.transform.rotation = msg.poses[1].orientation
        self.tf_broadcaster.sendTransform([t_left, t_right])

    # For debug of tf
    def update_real_ee_tf(self):
        if not self.tf_buffer.can_transform(
            self.ik_config.ROBOT_ROOT_FRAME,
            self.ik_config.LEFT_EE_TF_FRAME,
            rclpy.time.Time()
        ):
            return
        try:
            self.left_ee_tf = self.tf_buffer.lookup_transform(
                    self.ik_config.ROBOT_ROOT_FRAME,       # source frame / axis to use
                    self.ik_config.LEFT_EE_TF_FRAME,       # position of target_frame to acquire
                    rclpy.time.Time())
            self.right_ee_tf = self.tf_buffer.lookup_transform(
                    self.ik_config.ROBOT_ROOT_FRAME,       # source frame / axis to use
                    self.ik_config.RIGHT_EE_TF_FRAME,      # position of target_frame to acquire
                    rclpy.time.Time())
            if self.ik_config.DEBUG:
                self.get_logger().info(f'Transform found from end_effector into {self.ik_config.ROBOT_ROOT_FRAME} frame: '
                                       f'Translation (left): x={self.left_ee_tf.transform.translation.x:.2f}, y={self.left_ee_tf.transform.translation.y:.2f}, z={self.left_ee_tf.transform.translation.z:.2f}'
                                       f'\nTranslation (right): x={self.right_ee_tf.transform.translation.x:.2f}, y={self.right_ee_tf.transform.translation.y:.2f}, z={self.right_ee_tf.transform.translation.z:.2f}')

        except tf2_ros.LookupException as e:
            self.get_logger().info(f"Could not lookup transform trunk to end effector: {e}")

    def get_sde_robot_config_package_dir(self):
        """Get share directory path to YAML file."""
        return get_package_share_directory('sde_robot_config')

    def kill_subprocess(self):
        try:
            os.killpg(os.getpgid(self.robot_state_publisher_node_process.pid), signal.SIGINT)
            self.get_logger().info("Successfully killed robot state publisher process.")
        except:
            self.get_logger().info("Robot state publisher process not found or already killed.")

def main(args=None):
    rclpy.init(args=args)
    node = ROS2IKSolverHandler()
    executor = MultiThreadedExecutor(num_threads=node.num_ROS2_threads)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info('Shutdown requested.')
    finally:
        executor.shutdown()
        # if node.robot_state_publisher_node_process:
        node.kill_subprocess()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
