# Copyright (c) 2026, Agency for Science, Technology and Research (A*STAR) All Rights Reserved.
# License TO-DO: Apache License, Version 2.0  
# Maintainer: Ng Yung Chuen, Email: ng_yung_chuen@a-star.edu.sg
# File: vr_teleoperation_interface.py
# Description: A VR Interface node that gets target poses from VR controllers.

import sys 
import yaml 
import time
import random
import threading
import numpy as np
import pinocchio as pin
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import tf2_ros

from .utils.ros2_handler import ROS2Handler, pinSE3_to_Pose
from .utils.t1_teleop_wrapper import T_ROBOT_OPENXR, T_OPENXR_ROBOT
from .utils.t1_teleop_wrapper import T1TeleopWrapper, T_mat_to_euler_angles


class VRTeleopInterfaceNode:
    def __init__(self):
        # Init ROS2 handler
        self.ROS2_handler_node = ROS2Handler()
        self.ROS2_handler_spin = threading.Thread(target=self.ROS2_handler_node.spin_thread, daemon=True)

        # --- TeleVuer Setup using the new Wrapper ---
        self.tv_wrapper = None

        # Initialize VR wrapper
        try:
            if self.ROS2_handler_node.vr_device == "metaquest3":
                # Use the new T1TeleopWrapper
                self.ee_starting_pos_error = self.ROS2_handler_node.teleop_config['metaquest3']['ee_starting_pos_error']
                if self.ROS2_handler_node.teleop_config['head_camera_stream'][self.ROS2_handler_node.sim_or_real]['enable']:
                    self.vr_cam_height = self.ROS2_handler_node.teleop_config['head_camera_stream'][self.ROS2_handler_node.sim_or_real]['height']
                    self.vr_cam_width = self.ROS2_handler_node.teleop_config['head_camera_stream'][self.ROS2_handler_node.sim_or_real]['width']
                    camera_stream_res = (self.vr_cam_height, self.vr_cam_width)
                    assert tuple(self.ROS2_handler_node.teleop_config['metaquest3']['img_shape']) == camera_stream_res, "img_shape does not match head_camera_stream resolution"
                self.tv_wrapper = T1TeleopWrapper(img_shape=(self.ROS2_handler_node.teleop_config['metaquest3']['img_shape'][0], self.ROS2_handler_node.teleop_config['metaquest3']['img_shape'][1]), 
                    display_fps=self.ROS2_handler_node.teleop_config['metaquest3']['display_fps'],
                    display_mode=self.ROS2_handler_node.teleop_config['metaquest3']['display_mode'], # ["immersive", "pass-through", "ego"]
                    zmq=self.ROS2_handler_node.teleop_config['metaquest3']['zmq'], 
                    webrtc=self.ROS2_handler_node.teleop_config['metaquest3']['webrtc']
                )
                self.ROS2_handler_node.logger.info('TeleVuerWrapper initialized. The server is starting in the background.')
        except Exception as e:
            self.ROS2_handler_node.logger.info(f"Failed to initialize TeleVuerWrapper: {e}")
            self.ROS2_handler_node.logger.info("Ensure VR headset is connected and SteamVR is running.")
            self.destroy_node()
            rclpy.shutdown()
            raise e

        # Initialise default image text embedding settings
        if self.ROS2_handler_node.enable_vr_cam_stream:
            self.text_font = cv2.FONT_HERSHEY_SIMPLEX
            self.text_scale = 0.5
            self.text_colour = (255, 255, 255)
            self.text_thickness = 1
            (text_w, text_h), baseline = cv2.getTextSize("a", self.text_font, self.text_scale, self.text_thickness)
            self.text_position1 = (10, self.vr_cam_height - baseline - 50)
            self.text_position2 = (10, self.vr_cam_height - baseline - 30)
            self.text_position3 = (10, self.vr_cam_height - baseline - 10)
        
        # data collection variables
        # all variables having interactions with ROS2 Handler will be managed by the handler class
        self.start_flag = False
        self.start_episode = False
        self.ROS2_handler_node.manual_collect_data = 0
        self.ROS2_handler_node.episode = 1

        # Fixed variable for timeout of popup text to display in VR
        self.vr_popup_text_timeout = rclpy.duration.Duration(seconds=2.0)
        self.popup_text = ""
        self.last_vr_text_update_time: rclpy.time.Time | None = None
        self.last_task_update_time = self.ROS2_handler_node.get_clock().now()
        
        self.last_log_time = self.ROS2_handler_node.get_clock().now()
        self.last_ep_start_time = self.ROS2_handler_node.get_clock().now()
        self.last_teleop_start_time = self.ROS2_handler_node.get_clock().now()

        self.null_action_list = ['Do nothing','Idle','Stop']

        self.ROS2_handler_node.logger.info('VR Teleop Node initialized. Proceeding to sample data from VR device.')

    def test_VR_connection_and_start_teleop(self):
        """
        Guides the user through connection, then starts the teleop loop.
        """
        self.ROS2_handler_node.logger.info("--- WAITING FOR VR CONNECTION ---")
        self.ROS2_handler_node.logger.info("Please start the VR application on your headset.")
        
        # =================================================================
        # NEW: Wait for the VR server to connect to the VR client
        # and start receiving valid pose data.
        # =================================================================
        while rclpy.ok():
            if self.tv_wrapper.has_valid_poses():
                self.ROS2_handler_node.logger.info("VR Connection Established! Controller data is valid.")
                break
            self.ROS2_handler_node.logger.info(f"Waiting for valid VR controller poses from {self.ROS2_handler_node.vr_device}...")
            time.sleep(0.5)  # Poll at 2Hz to be CPU-friendly

        self.vr_poll_timer = self.ROS2_handler_node.create_timer(1.0 / self.ROS2_handler_node.teleop_config['poll_vr_frequency'], self.poll_vr_and_publish_pose_target, callback_group=self.ROS2_handler_node.cb_group)
        self.ROS2_handler_spin.start()

    def set_popup_text(self, popup_text, colour):
        self.popup_text = popup_text
        if colour == 'red':
            if "bgr" in self.ROS2_handler_node.rgb_img_encoding:
                self.popup_text_color = (0, 50, 255)
            else:
                self.popup_text_color = (255, 50, 0)
        if colour == 'green':
            self.popup_text_color = (0, 255, 0)
        self.last_vr_text_update_time = self.ROS2_handler_node.get_clock().now()

    def poll_vr_and_publish_pose_target(self):
        """Main control loop for polling VR headset to receive inputs."""
        # Remove text if timeout
        now = self.ROS2_handler_node.get_clock().now()
        if self.last_vr_text_update_time is not None:
            if now - self.last_vr_text_update_time <= self.vr_popup_text_timeout:
                (tmp_width, tmp_height), baseline = cv2.getTextSize(self.popup_text, self.text_font, 0.6, self.text_thickness)
                text_position4 = ((self.ROS2_handler_node.img_data.shape[1] - tmp_width) // 2, self.vr_cam_height - 80)
                cv2.putText(
                    self.ROS2_handler_node.img_data,
                    self.popup_text,
                    text_position4,
                    self.text_font,
                    0.6,                     # text_scale
                    self.popup_text_color,   # text_colour: red
                    self.text_thickness,
                    cv2.LINE_AA
                )
            else:
                self.last_vr_text_update_time = None
                self.ROS2_handler_node.logger.info('Warning timeout.')

        head_tf_target, L_tf_target, R_tf_target = self.tv_wrapper.get_motion_state_data()
        tele_button_data = self.tv_wrapper.get_button_state_data()
        if self.ROS2_handler_node.enable_vr_cam_stream:
            # if DEBUG:
            # print("Frame type:", type(self.ROS2_handler_node.img_data))
            # print("Frame shape:", getattr(self.ROS2_handler_node.img_data, "shape", None))
            # print("Frame dtype:", getattr(self.ROS2_handler_node.img_data, "dtype", None))
            if self.start_flag:
                if self.start_episode:
                    text = "Episode started"
                    if "bgr" in self.ROS2_handler_node.rgb_img_encoding:
                        text_colour = (200, 0, 0)
                    else:
                        text_colour = (0, 0, 200)
                else: 
                    text = "Teleop started"
                    text_colour = (0, 200, 0)
            else:
                text = "Teleop paused"
                if "bgr" in self.ROS2_handler_node.rgb_img_encoding:
                    text_colour = (0, 165, 255) # BGR
                else:
                    text_colour = (255, 165, 0) # RGB
            with self.ROS2_handler_node.cam_stream_lock:
                cv2.putText(
                    self.ROS2_handler_node.img_data,
                    text,
                    self.text_position1,
                    self.text_font,
                    self.text_scale,
                    text_colour,
                    self.text_thickness,
                    cv2.LINE_AA
                )
                cv2.putText(
                    self.ROS2_handler_node.img_data,
                    f"Episode: {self.ROS2_handler_node.episode}",
                    self.text_position2,
                    self.text_font,
                    self.text_scale,
                    self.text_colour,
                    self.text_thickness,
                    cv2.LINE_AA
                )
                cv2.putText(
                    self.ROS2_handler_node.img_data,
                    f"Task: {self.ROS2_handler_node.task_label}",
                    self.text_position3,
                    self.text_font,
                    self.text_scale,
                    self.text_colour,
                    self.text_thickness,
                    cv2.LINE_AA
                )
                self.tv_wrapper.render_to_xr(self.ROS2_handler_node.img_data, self.ROS2_handler_node.rgb_img_encoding)
        
        if self.ROS2_handler_node.verbose:
            self.ROS2_handler_node.logger.info(f"Left Target Pose: {L_tf_target}") # base_link (0., 0., 0.) is at 0.36 m below eye level
            self.ROS2_handler_node.logger.info(f"Right Target Pose: {R_tf_target}")
            # self.get_logger().info(f"Head Target Pose: {head_tf_target}", throttle_duration_sec=1) # DO NOT USE THROTTLE
            if (self.ROS2_handler_node.get_clock().now() - self.last_log_time).nanoseconds * 1e-9 > 1.0:
                self.ROS2_handler_node.logger.info(f"START BUTTON: {tele_button_data.left_ctrl_bButton}")
                self.last_log_time = self.ROS2_handler_node.get_clock().now()
        
        '''
        Head Target Pose: [[ 0.93524236 -0.2227973  -0.27510556 -0.01673685]
        [vr_ik_solver-4]  [ 0.19749535  0.97331488 -0.11684931  0.05671664]
        [vr_ik_solver-4]  [ 0.29379803  0.05495035  0.95428675  1.09406159]
        [vr_ik_solver-4]  [ 0.          0.          0.          1.        ]]

        '''

        # Yet to receive data from VR headset
        if L_tf_target is None or R_tf_target is None:
            if (self.ROS2_handler_node.get_clock().now() - self.last_log_time).nanoseconds * 1e-9 > 1.0:
                self.ROS2_handler_node.logger.info("Waiting for valid controller poses...")
                self.last_log_time = self.ROS2_handler_node.get_clock().now()
            return

        # Change Task
        if tele_button_data.left_ctrl_thumbstick: 
            if self.start_episode is True: 
                self.ROS2_handler_node.logger.info("Please do not change task in the middle of an episode.")
                self.set_popup_text("Do not change task midst episode", "red")
            else:
                if (self.ROS2_handler_node.get_clock().now() - self.last_task_update_time).nanoseconds * 1e-9 > 0.5:
                    self.ROS2_handler_node.task_labelling_id = (self.ROS2_handler_node.task_labelling_id - 1) % self.ROS2_handler_node.task_labelling_list_len
                    self.ROS2_handler_node.task_label = self.ROS2_handler_node.task_labelling_list[self.ROS2_handler_node.task_labelling_id]
                    self.ROS2_handler_node.logger.info(f"Task now is: {self.ROS2_handler_node.task_label}.")
                    self.last_task_update_time = self.ROS2_handler_node.get_clock().now()
        if tele_button_data.right_ctrl_thumbstick: 
            if self.start_episode is True: 
                self.ROS2_handler_node.logger.info("Please do not change task in the middle of an episode.")
                self.set_popup_text("Do not change task midst episode", "red")
            else:
                if (self.ROS2_handler_node.get_clock().now() - self.last_task_update_time).nanoseconds * 1e-9 > 0.5:
                    self.ROS2_handler_node.task_labelling_id = (self.ROS2_handler_node.task_labelling_id + 1) % self.ROS2_handler_node.task_labelling_list_len
                    self.ROS2_handler_node.task_label = self.ROS2_handler_node.task_labelling_list[self.ROS2_handler_node.task_labelling_id]
                    self.ROS2_handler_node.logger.info(f"Task now is: {self.ROS2_handler_node.task_label}.")
                    self.last_task_update_time = self.ROS2_handler_node.get_clock().now()

        # Open or Close Gripper
        if self.ROS2_handler_node.enable_gripper:
        # left gripper 
            if tele_button_data.left_ctrl_trigger and tele_button_data.left_ctrl_squeeze:
                with self.ROS2_handler_node.gripper_lock:
                    self.ROS2_handler_node.gripper_state_msg.data[0] = int((tele_button_data.left_ctrl_triggerValue / 10 * 400) + ((1 - tele_button_data.left_ctrl_squeezeValue) * 400))
            else:
                with self.ROS2_handler_node.gripper_lock:
                    self.ROS2_handler_node.gripper_state_msg.data[0] = self.ROS2_handler_node.gripper_max_pos

            # right gripper
            if tele_button_data.right_ctrl_trigger and tele_button_data.right_ctrl_squeeze:
                with self.ROS2_handler_node.gripper_lock:
                    self.ROS2_handler_node.gripper_state_msg.data[1] = int((tele_button_data.right_ctrl_triggerValue / 10 * 400) + ((1 - tele_button_data.right_ctrl_squeezeValue) * 400))
            else:
                with self.ROS2_handler_node.gripper_lock:
                    self.ROS2_handler_node.gripper_state_msg.data[1] = self.ROS2_handler_node.gripper_max_pos

        # Start Teleop
        if tele_button_data.left_ctrl_bButton: 
            if self.start_flag == True:
                if (self.ROS2_handler_node.get_clock().now() - self.last_teleop_start_time).nanoseconds * 1e-9 > 1.0:
                        if self.ROS2_handler_node.task_label not in self.null_action_list:
                            self.ROS2_handler_node.task_label = random.choice(self.null_action_list)
                            self.ROS2_handler_node.is_done = True
                            self.ROS2_handler_node.logger.info("Task set to idle.")
                            self.set_popup_text("Task set to idle.", "red")
                        else:
                            self.ROS2_handler_node.task_label = self.ROS2_handler_node.task_labelling_list[self.ROS2_handler_node.task_labelling_id]
                            self.ROS2_handler_node.is_done = False
                            self.ROS2_handler_node.logger.info(f"Task set to {self.ROS2_handler_node.task_label}.")
                            self.set_popup_text("Task resume.", "green")
                        self.last_teleop_start_time = self.ROS2_handler_node.get_clock().now()
                else:
                    self.ROS2_handler_node.logger.info("Teleop already started.")
            else:
                # Check if left and right target pose is in the last known place before pause or starting pos
                L_tf_current = np.array([self.ROS2_handler_node.left_ee_tf.transform.translation.x, self.ROS2_handler_node.left_ee_tf.transform.translation.y, self.ROS2_handler_node.left_ee_tf.transform.translation.z])
                R_tf_current = np.array([self.ROS2_handler_node.right_ee_tf.transform.translation.x, self.ROS2_handler_node.right_ee_tf.transform.translation.y, self.ROS2_handler_node.right_ee_tf.transform.translation.z])
                # self.ROS2_handler_node.logger.info(f"L_tf_target.translation VR: {L_tf_target.translation}")
                # self.ROS2_handler_node.logger.info(f"L_tf_current.translation TF: {L_tf_current}")
                # self.ROS2_handler_node.logger.info(f"R_tf_target.translation VR: {R_tf_target.translation}")
                # self.ROS2_handler_node.logger.info(f"R_tf_current.translation TF: {R_tf_current}")
                # self.ROS2_handler_node.logger.info(f"head_pose.translation VR: {head_tf_target.translation}")
                check_left_pos_distance = np.linalg.norm(L_tf_target.translation - L_tf_current)
                check_right_pos_distance = np.linalg.norm(R_tf_target.translation - R_tf_current)
                if self.ROS2_handler_node.verbose:
                    self.ROS2_handler_node.logger.info("check_left_pos_distance:", check_left_pos_distance)
                    self.ROS2_handler_node.logger.info("check_right_pos_distance:", check_right_pos_distance)
                if (check_left_pos_distance <= self.ee_starting_pos_error) and (check_right_pos_distance <= self.ee_starting_pos_error):
                    self.start_flag = True
                    self.ROS2_handler_node.logger.info("Teleop started.")
                    self.last_teleop_start_time = self.ROS2_handler_node.get_clock().now()
                else:
                    self.ROS2_handler_node.logger.info("Either hand is not near neutral position, unable to start.")
                    self.set_popup_text("Hands not near neutral position, start fail", "red")
                
        # Pause Teleop
        if tele_button_data.left_ctrl_aButton: 
            # Recording is not started, just pause
            if self.start_episode is False: 
                self.start_flag = False
                self.ROS2_handler_node.logger.info(f"Teleop paused.")
            # Recording is started, discard episode
            if self.start_episode is True: 
                self.start_episode = False
                if self.ROS2_handler_node.collect_data:
                    self.ROS2_handler_node.send_collect_data_service("discard", self.ROS2_handler_node.episode)
                self.ROS2_handler_node.logger.info(f"Discarding episode {self.ROS2_handler_node.episode}.")
                self.set_popup_text(f"Discarding episode {self.ROS2_handler_node.episode}", "red")

        # Only when started, allow ik computation and bag record
        if self.start_flag:
            # Start Episode
            if tele_button_data.right_ctrl_bButton or self.ROS2_handler_node.manual_collect_data == 1: 
                if not self.start_episode:
                    self.start_episode = True
                    self.ROS2_handler_node.is_done = False
                    self.last_ep_start_time = self.ROS2_handler_node.get_clock().now()
                    # ROS2 service call to start data collection in ROS2 bag
                    if self.ROS2_handler_node.collect_data:
                        self.ROS2_handler_node.send_collect_data_service("start", self.ROS2_handler_node.episode)
                    self.ROS2_handler_node.logger.info(f"Start episode {self.ROS2_handler_node.episode}.")
                else:
                    if (self.ROS2_handler_node.get_clock().now() - self.last_ep_start_time).nanoseconds * 1e-9 > 1.0:
                        if not self.ROS2_handler_node.is_done:
                            self.ROS2_handler_node.task_label = random.choice(self.null_action_list)
                            self.ROS2_handler_node.is_done = True
                            self.ROS2_handler_node.logger.info(f"Episode {self.ROS2_handler_node.episode} set to complete. Please return to neutral position before ending episode.")
                            self.set_popup_text(f"Episode {self.ROS2_handler_node.episode} set to completed", "green")
                        else:
                            self.ROS2_handler_node.task_label = self.ROS2_handler_node.task_labelling_list[self.ROS2_handler_node.task_labelling_id]
                            self.ROS2_handler_node.is_done = False
                            self.ROS2_handler_node.logger.info(f"Episode {self.ROS2_handler_node.episode} not complete. Please proceed with the task.")
                            self.set_popup_text(f"Episode {self.ROS2_handler_node.episode} not completed, please continue task", "red")
                        self.last_ep_start_time = self.ROS2_handler_node.get_clock().now()
                    else:
                        self.ROS2_handler_node.logger.info(f"Episode {self.ROS2_handler_node.episode} already started. Press X to discard this episode to reset.")
                        # self.set_popup_text(f"Episode {self.ROS2_handler_node.episode} already started", "red") # maybe don't need
            
            # Stop Episode
            if tele_button_data.right_ctrl_aButton or self.ROS2_handler_node.manual_collect_data == 2: 
                if self.start_episode:
                    self.start_episode = False
                    # ROS2 service call to stop data collection in ROS2 bag
                    if self.ROS2_handler_node.collect_data:
                        self.ROS2_handler_node.send_collect_data_service("stop", self.ROS2_handler_node.episode)
                    self.ROS2_handler_node.logger.info(f"End episode {self.ROS2_handler_node.episode}.")
                    self.set_popup_text(f"Saved episode {self.ROS2_handler_node.episode}", "green")
                    self.ROS2_handler_node.episode += 1
                    self.ROS2_handler_node.is_done = False
                    self.ROS2_handler_node.task_label = self.ROS2_handler_node.task_labelling_list[self.ROS2_handler_node.task_labelling_id]
                else:
                    self.ROS2_handler_node.logger.info(f"Episode {self.ROS2_handler_node.episode} has not yet started, please press B to start the episode.")
                    # self.set_popup_text(f"Episode {self.ROS2_handler_node.episode} has not yet started", "red")
            
            self.ROS2_handler_node.manual_collect_data = 0

            pose_array = []
            try:
                if self.ROS2_handler_node.verbose:
                    self.ROS2_handler_node.logger.info("Received valid target poses from VR controllers.")

                pose_array.append(pinSE3_to_Pose(L_tf_target))
                pose_array.append(pinSE3_to_Pose(R_tf_target))
                pose_array.append(pinSE3_to_Pose(head_tf_target))

                self.ROS2_handler_node.publish_target_poses(pose_array)
                
            except Exception as e:
                if (self.ROS2_handler_node.get_clock().now() - self.last_log_time).nanoseconds * 1e-9 > 5.0:
                    self.ROS2_handler_node.logger.info(f"Failed to publish pose target during VR poll: {e}")
                    self.last_log_time = self.ROS2_handler_node.get_clock().now()
        else:
            if (self.ROS2_handler_node.get_clock().now() - self.last_log_time).nanoseconds * 1e-9 > 1.0:
                self.ROS2_handler_node.logger.info("Please assume neutral position and click on the left Y button to commence teleop.")
                self.last_log_time = self.ROS2_handler_node.get_clock().now()
            # self.set_popup_text("Click Y to start in last known pose", "red")
            return

        # -----------------------------------------------------------------
        #  Debug-marker publishing (target, raw VR controller, calib Xfms)
        # -----------------------------------------------------------------
        if self.ROS2_handler_node.vr_device == "metaquest3":
            raw_L_xr = self.tv_wrapper.tvuer.left_arm_pose
            raw_R_xr = self.tv_wrapper.tvuer.right_arm_pose
            vr_L_robot = pin.SE3(T_ROBOT_OPENXR @ raw_L_xr @ T_OPENXR_ROBOT)
            vr_R_robot = pin.SE3(T_ROBOT_OPENXR @ raw_R_xr @ T_OPENXR_ROBOT)

        self.ROS2_handler_node.publish_debug_markers(
            L_tf_target, R_tf_target,
            vr_L_robot, vr_R_robot,
            self.tv_wrapper.T_robotbase_vrworld_L,
            self.tv_wrapper.T_robotbase_vrworld_R)

        self.ROS2_handler_node.publish_task_label(self.ROS2_handler_node.task_label)
        self.ROS2_handler_node.publish_episode_idx(self.ROS2_handler_node.episode)
        self.ROS2_handler_node.publish_is_done(self.ROS2_handler_node.is_done)

    def destroy_node(self):
        self.ROS2_handler_node.logger.info("Shutting down and cleaning up resources.")
        if self.ROS2_handler_node.vr_device == "metaquest3":
            if self.tv_wrapper and self.tv_wrapper.tvuer.process.is_alive():
                 self.ROS2_handler_node.logger.info(f"Terminating {self.ROS2_handler_node.vr_device} process...")
                 self.tv_wrapper.tvuer.process.terminate()
                 self.tv_wrapper.tvuer.process.join(timeout=2)

        if self.tv_wrapper:
            self.tv_wrapper.close()

        try:
            self.ROS2_handler_spin.join()
        except:
            print("ROS2 thread not started, no threads to join.")
        self.ROS2_handler_node.destroy_node()
        print("Shutdown successful.")

def main(args=None):
    rclpy.init(args=args)
    vr_node = None
    try:
        vr_node = VRTeleopInterfaceNode()

        # Run connection wait and calibration in the main thread.
        vr_node.test_VR_connection_and_start_teleop()
        
        # Keep the main thread alive to handle Ctrl+C
        while rclpy.ok():
            time.sleep(1)

    except KeyboardInterrupt:
        if vr_node: vr_node.ROS2_handler_node.logger.info('KeyboardInterrupt, shutting down.')
    except Exception as e:
        if vr_node: vr_node.ROS2_handler_node.logger.info(f"An unhandled exception occurred: {e}")
        else: print(f"An unhandled exception occurred during node initialization: {e}")
    finally:
        if vr_node:
            vr_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

