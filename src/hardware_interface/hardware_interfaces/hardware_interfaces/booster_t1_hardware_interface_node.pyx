# Copyright (c) 2026, Agency for Science, Technology and Research (A*STAR) All Rights Reserved.
# License TO-DO: Apache License, Version 2.0  
# Maintainer: Brina Shong, Email: brina_shong@a-star.edu.sg

# TO-DO: if possible, make code generalisable to other humanoid robot embodiment by loading config file?

import os
import yaml
import signal
import sys
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray, Int64MultiArray
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose
from booster_interface.msg import LowCmd, MotorCmd, LowState, MotorState
from booster_robotics_sdk_python import B1LocoClient, ChannelFactory, GripperMotionParameter, GripperControlMode, B1HandIndex
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np 
import time

class JointLimits:
    def __init__(self, min_pos, max_pos):
        self.min_position = min_pos
        self.max_position = max_pos

class HardwareInterface(Node):
    def __init__(self):
        super().__init__('Booster_T1_hardware_interface_node')

        # register signal handler
        signal.signal(signal.SIGINT, self.sigint_handler)

        # Thread safety
        # self.mutex = threading.Lock()

        # Parameters
        self.declare_parameter('cmd_type', LowCmd.CMD_TYPE_SERIAL)
        self.declare_parameter('client_ip', "127.0.0.1")
        self.declare_parameter('config_name', "booster_t1_hardware_interface.yaml")

        ChannelFactory.Instance().Init(0, self.get_parameter('client_ip').value)
        self.client = B1LocoClient()
        self.client.Init()

        self.cmd_type = self.get_parameter('cmd_type').value

        # Joint limits dictionary
        self.joint_limits = {}

        # Load joint limits from YAML
        if not self.load_config():
            raise RuntimeError("Failed to load config file, cannot continue")

        # Publisher - LowCmd for control
        self.ctrl_pub = self.create_publisher(
            LowCmd,
            '/joint_ctrl',
            10
        )

        self.real_joint_pub = self.create_publisher(
            JointState,
            '/real/joint_states',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/real/imu',
            10
        )

        self.joint_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/real/joint_vel',
            10
        )

        self.head_rpy_pub = self.create_publisher(
            Float64MultiArray,
            '/real/head_rpy',
            10
        )

        self.sensor_group = ReentrantCallbackGroup()

        # Subscriber - JointState input
        self.joint_sub = self.create_subscription(
            JointState,
            '/ik/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.sensor_group
        )

        self.state_sub = self.create_subscription(
            LowState,
            '/low_state',
            self.low_state_callback,
            10,
            callback_group=self.sensor_group
        )

        self.head_pose_sub = self.create_subscription(
            Pose,
            '/head_pose',
            self.head_pose_callback,
            10,
            callback_group=self.sensor_group
        )

        self.hand_states_sub = self.create_subscription(
            Int64MultiArray,
            '/hand_states',
            self.hand_states_callback,
            10,
            callback_group=self.sensor_group
        )

        # self.get_logger().info("Joint State to Control Interface Started")
        print("Joint State to Control Interface Started")
        self.first_iter = True
        self.curr_joint_state = None

        self.left_gripper = -1
        self.right_gripper = -1

    def sigint_handler(self, signum, frame):
        """
        Called automatically when user presses Ctrl+C.
        """
        # self.get_logger().info("Ctrl+C detected! Running cleanup...")
        print("Ctrl+C detected! Running cleanup...")

        # Run cleanup tasks
        self.cleanup()

        # After cleanup, shutdown ROS2
        rclpy.shutdown()

    def cleanup(self):
        # self.get_logger().info("Cleaning up resources...")
        print("Cleaning up resources...")
        self.init_pos(False)

    def init_pos(self, init=True):
        """Initialise robot and place robot in neutral arm position"""
        print("Running init pose.")
        if not self.curr_joint_state:
            print("Not yet receive initial robot joint state.")
            return
        current_jpos = self.curr_joint_state
        if init:
            target_jpos = np.array(self.start_configuration)
        else: 
            target_jpos = np.array(self.end_configuration)

        def linear_interpolate(start_jpos, end_jpos, t_to_initialise=3.0, dt=0.01):
            """Linearly interpolate start jpos to end_jpos with n_steps"""
            num_steps = int(t_to_initialise / dt) + 1
            times = np.linspace(0, t_to_initialise, num_steps+1)
            trajectories = start_jpos + (end_jpos - start_jpos) * (times[:, None] / t_to_initialise)

            return times, trajectories

        trajectories = linear_interpolate(current_jpos, target_jpos)

        for traj in trajectories[1]: 
            # Create LowCmd message
            cmd_msg = LowCmd()

            cmd_msg.cmd_type = self.cmd_type

            # Initialize motor_cmd array with empty MotorCmd messages
            # Create one for each motor in the mapping
            num_motors = max(self.joint_mapping.values()) + 1 if self.joint_mapping else 10
            cmd_msg.motor_cmd = []

            for i in range(num_motors):
                motor_cmd = MotorCmd()
                motor_cmd.mode = 0      # position control mode
                motor_cmd.q = float(traj[i])   # directly assign as already in order
                motor_cmd.dq = 0.0      # no velocity command
                motor_cmd.tau = 0.0     # no torque command
                motor_cmd.kp = self.joint_kps[i]
                motor_cmd.kd = self.joint_kds[i]
                motor_cmd.weight = 1.0  # full weight
                cmd_msg.motor_cmd.append(motor_cmd)

            self.ctrl_pub.publish(cmd_msg)
            time.sleep(0.01)

        self.first_iter = False

    def load_config(self):
        """Load configurations from YAML configuration file"""
        try:
            # Get the path to the robot configurations YAML file
            config_name = self.get_parameter('config_name').value
            package_share_dir = get_package_share_directory('hardware_interfaces')
            config_file = os.path.join(package_share_dir, 'config', config_name)

            # self.get_logger().info(f"Loading configs from: {config_file}")
            print(f"Loading configs from: {config_file}")

            # Load the YAML file
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)

            # Joint name to motor index mapping
            if 'joint_mapping' not in config:
                # self.get_logger().info(f"No 'joint_mapping' section found in {config_file}")
                print(f"No 'joint_mapping' section found in {config_file}")
                return False

            self.joint_mapping = config['joint_mapping']
            # self.get_logger().info(f"Successfully loaded joint_mapping: {self.joint_mapping}")
            print(f"Successfully loaded joint_mapping: {self.joint_mapping}")

            if 'joint_limits' not in config:
                # self.get_logger().info(f"No 'joint_limits' section found in {config_file}")
                print(f"No 'joint_limits' section found in {config_file}")
                return False

            joint_limits_config = config['joint_limits']

            # Parse joint limits for each joint in the mapping
            for joint_name in self.joint_mapping.keys():
                if joint_name in joint_limits_config:
                    limits_data = joint_limits_config[joint_name]
                    if 'min_position' in limits_data and 'max_position' in limits_data:
                        limits = JointLimits(
                            limits_data['min_position'],
                            limits_data['max_position']
                        )
                        self.joint_limits[joint_name] = limits

                        # self.get_logger().info(
                        #     f"Loaded limits for joint {joint_name}: "
                        #     f"min={limits.min_position:.3f}, max={limits.max_position:.3f}"
                        # )
                        print(
                            f"Loaded limits for joint {joint_name}: "
                            f"min={limits.min_position:.3f}, max={limits.max_position:.3f}"
                        )
                    else:
                        # self.get_logger().info(f"Incomplete limits for joint: {joint_name}")
                        print(f"Incomplete limits for joint: {joint_name}")
                else:
                    # self.get_logger().info(f"No limits found for joint: {joint_name}")
                    print(f"No limits found for joint: {joint_name}")

            # self.get_logger().info(
            #     f"Successfully loaded joint limits for {len(self.joint_limits)} joints"
            # )
            print(
                f"Successfully loaded joint limits for {len(self.joint_limits)} joints"
            )

            if 'start_configuration' not in config:
                # self.get_logger().info(f"No 'start_configuration' section found in {config_file}")
                print(f"No 'start_configuration' section found in {config_file}")
                return False
            self.start_configuration = config['start_configuration']

            if 'end_configuration' not in config:
                # self.get_logger().info(f"No 'end_configuration' section found in {config_file}")
                print(f"No 'end_configuration' section found in {config_file}")
                return False
            self.end_configuration = config['end_configuration']

            if 'joint_kps' not in config:
                # self.get_logger().info(f"No 'joint_kps' section found in {config_file}")
                print(f"No 'joint_kps' section found in {config_file}")
                return False
            self.joint_kps = config['joint_kps']

            if 'joint_kds' not in config:
                # self.get_logger().info(f"No 'joint_kds' section found in {config_file}")
                print(f"No 'joint_kds' section found in {config_file}")
                return False
            self.joint_kds = config['joint_kds']

            # self.get_logger().info(
            #     f"Successfully loaded T1 configurations."
            # )
            print(
                f"Successfully loaded T1 configurations."
            )

            return True

        except FileNotFoundError:
            # self.get_logger().info("Joint limits file not found")
            print("Joint limits file not found")
            return False
        except yaml.YAMLError as e:
            # self.get_logger().info(f"Failed to parse YAML file: {e}")
            print(f"Failed to parse YAML file: {e}")
            return False
        except Exception as e:
            # self.get_logger().info(f"Failed to load parameters: {e}")
            print(f"Failed to load parameters: {e}")
            return False


    def clamp_position(self, joint_name, position):
        """Clamp joint position to limits"""
        if joint_name in self.joint_limits:
            limits = self.joint_limits[joint_name]
            clamped = max(limits.min_position, min(limits.max_position, position))

            if clamped != position:
                # self.get_logger().info(
                #     f"Joint {joint_name} position {position:.3f} clamped to {clamped:.3f}",
                #     throttle_duration_sec=5.0
                # )
                print(
                    f"Joint {joint_name} position {position:.3f} clamped to {clamped:.3f}",
                )

            return clamped
        else:
            # No limits available, return original position
            return position


    def joint_state_callback(self, msg):
        """
        Convert JointState to JointCtrl message
        """
        # Robot has not been initialised, do not publish joint control
        if self.first_iter is True:
            self.init_pos(True)
            return

        # Create LowCmd message
        cmd_msg = LowCmd()

        cmd_msg.cmd_type = self.cmd_type

        # Initialize motor_cmd array with empty MotorCmd messages
        # Create one for each motor in the mapping
        num_motors = max(self.joint_mapping.values()) + 1 if self.joint_mapping else 10
        cmd_msg.motor_cmd = []

        for i in range(num_motors):
            motor_cmd = MotorCmd()
            motor_cmd.mode = 0      # position control mode
            motor_cmd.q = 0.0
            motor_cmd.dq = 0.0      # no velocity command
            motor_cmd.tau = 0.0     # no torque command
            motor_cmd.kp = self.joint_kps[i]
            motor_cmd.kd = self.joint_kds[i]
            motor_cmd.weight = 1.0  # full weight
            cmd_msg.motor_cmd.append(motor_cmd)

        # Map joints by name
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_mapping:
                motor_idx = self.joint_mapping[joint_name]

                if motor_idx < len(cmd_msg.motor_cmd):
                    # check limits
                    clamped_position = self.clamp_position(joint_name, msg.position[i])
                    cmd_msg.motor_cmd[motor_idx].q = float(clamped_position)

        self.ctrl_pub.publish(cmd_msg)


    def low_state_callback(self, msg):
        # Validate serial motor state size
        if len(msg.motor_state_serial) != 29:
            # self.get_logger().info(
            #     f"Invalid serial motor state size: {len(msg.motor_state_serial)} (expected 29)"
            # )
            print(
                f"Invalid serial motor state size: {len(msg.motor_state_serial)} (expected 29)"
            )
            return None

        # Initialize result dictionary
        imu = Imu()
        joint_states = JointState()
        joint_vel = Float64MultiArray()

        imu.header.stamp = self.get_clock().now().to_msg()
        joint_states.header.stamp = self.get_clock().now().to_msg()

        # Imu
        quaternion = quaternion_from_euler(msg.imu_state.rpy[0], msg.imu_state.rpy[1], msg.imu_state.rpy[2])
        imu.orientation.x = float(quaternion[0])
        imu.orientation.y = float(quaternion[1])
        imu.orientation.z = float(quaternion[2])
        imu.orientation.w = float(quaternion[3])
        imu.angular_velocity.x = float(msg.imu_state.gyro[0])
        imu.angular_velocity.y = float(msg.imu_state.gyro[1])
        imu.angular_velocity.z = float(msg.imu_state.gyro[2])
        imu.linear_acceleration.x = float(msg.imu_state.acc[0])
        imu.linear_acceleration.y = float(msg.imu_state.acc[1])
        imu.linear_acceleration.z = float(msg.imu_state.acc[2])

        # Extract data from serial motor states
        for joint_name, i in self.joint_mapping.items():

            # Joint State
            joint_states.name.append(joint_name)
            joint_states.position.append(msg.motor_state_serial[i].q)
            joint_states.velocity.append(msg.motor_state_serial[i].dq)
            joint_states.effort.append(msg.motor_state_serial[i].tau_est)

            # Joint Velocities
            joint_vel.data.append(msg.motor_state_serial[i].dq)

        self.curr_joint_state = joint_states.position

        self.imu_pub.publish(imu)
        self.real_joint_pub.publish(joint_states)
        self.joint_vel_pub.publish(joint_vel)

    def head_pose_callback(self, msg):
        head_rpy = Float64MultiArray()

        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

        # Convert to RPY
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        head_rpy.data.append(float(roll))
        head_rpy.data.append(float(pitch))
        head_rpy.data.append(float(yaw))

        self.head_rpy_pub.publish(head_rpy)

    def hand_states_callback(self, msg):
        if (len(msg.data) != 2):
            # self.get_logger().info(
            #     f"Invalid hand states size: {len(msg.data)} (expected 2)"
            # )
            print(
                f"Invalid hand states size: {len(msg.data)} (expected 2)"
            )
            return None

        # TO-DO: not make it hardcoded
        motion_param = GripperMotionParameter()
        motion_param.force = 100
        # motion_param.position = 100
        motion_param.speed = 100

        # motion_param.force = msg.data[0]
        if (msg.data[0] != self.left_gripper):
            self.left_gripper = msg.data[0]
            motion_param.position = msg.data[0]
            res = self.client.ControlGripper(motion_param, GripperControlMode.kPosition, B1HandIndex.kLeftHand)

            if res != 0:
                # self.get_logger().info(f"Request left gripper failed: error = {res}")
                print(f"Request left gripper failed: error = {res}")
            else:
                # self.get_logger().info(f"Request left gripper: {msg.data[0]} succeeded!")
                print(f"Request left gripper: {msg.data[0]} succeeded!")


        # motion_param.force = msg.data[1]
        if (msg.data[1] != self.right_gripper):
            self.right_gripper = msg.data[1]
            motion_param.position = msg.data[1]
            res = self.client.ControlGripper(motion_param, GripperControlMode.kPosition, B1HandIndex.kRightHand)

            if res != 0:
                # self.get_logger().info(f"Request right gripper failed: error = {res}")
                print(f"Request right gripper failed: error = {res}")
            else: 
                # self.get_logger().info(f"Request right gripper: {msg.data[1]} succeeded!")
                print(f"Request right gripper: {msg.data[1]} succeeded!")

def main():
    rclpy.init()
    node = HardwareInterface()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
