# -----------------------------------------------------------
# File: ik_solver_node.py
# Description: A demonstration IK solver node that subscribes to a PoseArray topic.
#
# Author: Hari Prasanth
# Date Created: 16 July 2025

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, TransformStamped
import tf2_ros
import pinocchio as pin
import numpy as np
import threading
from casadi_optimal_control_ik.t1_ik_core_collision import RobotArmIK, IKConfig

class DemoIKSolverNode(Node):
    def __init__(self):
        super().__init__('t1_demo_ik_collision_solver_node')
        self.get_logger().info('Initializing T1 Demo IK Collision Solver Node...')

         
        self.declare_parameter(
            'config_file_path',
            '',
            ParameterDescriptor(
                description='Path to YAML config for ik solver'
            )
        )
        
        config_file_path = self.get_parameter('config_file_path').get_parameter_value().string_value

        try:
            package_share = get_package_share_directory('sde_robot_config')
            config_file_path = os.path.join(package_share, 'config/booster_t1/ik_solver_config.yaml')
        except Exception:
            raise FileNotFoundError("Could not find default config file. Please check config_file_path.")

        self.arm_ik = RobotArmIK(config_file_path)

        self.ik_config = self.arm_ik.config

        self.pose_estimator_sub = self.create_subscription(
            JointState,
            '/pose_estimator/joint_states',  # The topic your pose estimator publishes to
            self.pose_estimator_callback,
            10
        )
        
        self.joint_pub  = self.create_publisher(
            JointState, 
            '/ik/joint_states', 
            self.ik_config.QOS_DEPTH
        )
        
        #self.target_sub = self.create_subscription(
        #    PoseArray, 
        #    '/hand_poses_target', 
        #    self.solve_ik_callback, 
        #    self.ik_config.QOS_DEPTH
        #)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.lock = threading.Lock()
        
        self.full_joint_names = [name for name in self.arm_ik.full_robot.model.names[1:]]
        reduced_joint_names = [name for name in self.arm_ik.reduced_robot.model.names[1:]]
        self.reduced_to_full_map = [self.full_joint_names.index(name) for name in reduced_joint_names]

        # for idx, name in enumerate(self.full_joint_names):
        #     if name == "Waist_joint":
        #         self.full_joint_names[idx] = "Waist"

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.full_joint_names
        ### Initialize joint state position with reference configuration
        # self.joint_state_msg.position = list(pin.neutral(self.arm_ik.full_robot.model))
        self.joint_state_msg.position = list(self.ik_config.REFERENCE_CONFIGURATION)
        
        self.timer = self.create_timer(1.0 / self.ik_config.PUBLISH_RATE_HZ, self.publish_joint_states)
        
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.ee_tf_timer = self.create_timer(1.0 / self.ik_config.EE_TF_SAMPLE_HZ, self.update_real_ee_tf)

        self.get_logger().info('Demo node initialized. Waiting for target poses on /hand_poses_target.')

    def pose_estimator_callback(self, msg):
        with self.lock:
            # For each joint in the incoming message, update the corresponding value in the full joint state
            for name, value in zip(msg.name, msg.position):
                if name in self.joint_state_msg.name:
                    idx = self.joint_state_msg.name.index(name)
                    self.joint_state_msg.position[idx] = value

    def publish_joint_states(self):
        with self.lock:
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.joint_state_msg)

    def solve_ik_callback(self, msg: PoseArray):
        if len(msg.poses) != 2:
            self.get_logger().info(f"Received PoseArray with {len(msg.poses)} poses, but expected 2. Ignoring.")
            return

        def pose_to_se3(pose):
            p = np.array([pose.position.x, pose.position.y, pose.position.z])
            q = pin.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            return pin.SE3(q.toRotationMatrix(), p)

        L_tf_target = pose_to_se3(msg.poses[0])
        R_tf_target = pose_to_se3(msg.poses[1])
        
        try:
            arm_solution_q = self.arm_ik.solve_ik(L_tf_target.homogeneous, R_tf_target.homogeneous)
            with self.lock:
                for i, q_val in enumerate(arm_solution_q):
                    full_model_index = self.reduced_to_full_map[i]
                    self.joint_state_msg.position[full_model_index] = q_val
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

    # FOR DEBUG OF TF
    def update_real_ee_tf(self):
        try:
            self.left_ee_tf = self.tf_buffer.lookup_transform(
                    self.ik_config.ROBOT_ROOT_FRAME,       # source frame / axis to use
                    "left_hand_link",                      # position of target_frame to acquire
                    rclpy.time.Time())
            self.right_ee_tf = self.tf_buffer.lookup_transform(
                    self.ik_config.ROBOT_ROOT_FRAME,       # source frame / axis to use
                    "right_hand_link",                     # position of target_frame to acquire
                    rclpy.time.Time())
            #self.get_logger().info(f'Transform found from end_effector into trunk frame: '
            #                       f'Translation (left): x={self.left_ee_tf.transform.translation.x:.2f}, y={self.left_ee_tf.transform.translation.y:.2f}, z={self.left_ee_tf.transform.translation.z:.2f}'
            #                       f'\nTranslation (right): x={self.right_ee_tf.transform.translation.x:.2f}, y={self.right_ee_tf.transform.translation.y:.2f}, z={self.right_ee_tf.transform.translation.z:.2f}')

        except tf2_ros.LookupException as e:
            self.get_logger().info(f"Could not lookup transform trunk to end effector: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DemoIKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
