#-----------------------------------------
# File: t1_teleop_wrapper.py
# Description: A wrapper for the TeleVuer library to handle VR data acquisition and coordinate frame transformations for the T1 robot.
# Date creatd: 18 July 2025


import numpy as np
import pinocchio as pin
from televuer import TeleVuer
from dataclasses import dataclass, field
from scipy.spatial.transform import Rotation
from typing import Literal

# TODO: 
# 1. Write proper comments on what each of the Transformation matrices do
# 2. Perform some matrix checks to ensure the the target pose commands are always valid

# Re-using the essential basis transformation from the original code
T_ROBOT_OPENXR = np.array([
    [ 0, 0, -1, 0],
    [-1, 0,  0, 0],
    [ 0, 1,  0, 0],
    [ 0, 0,  0, 1]
])
T_OPENXR_ROBOT = np.linalg.inv(T_ROBOT_OPENXR)

def safe_mat_update(prev_mat, mat):
    # Return previous matrix and False flag if the new matrix is non-singular (determinant ≠ 0).
    det = np.linalg.det(mat)
    if not np.isfinite(det) or np.isclose(det, 0.0, atol=1e-6):
        return prev_mat, False
    return mat, True

def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret

def safe_rot_update(prev_rot_array, rot_array):
    dets = np.linalg.det(rot_array)
    if not np.all(np.isfinite(dets)) or np.any(np.isclose(dets, 0.0, atol=1e-6)):
        return prev_rot_array, False
    return rot_array, True

def T_mat_to_euler_angles(T_mat, sequence='zyx', in_deg=False):
    """
    Converts a 4x4 homogeneous transformation matrix to Euler angles.

    Args:
        T_mat (np.ndarray): A 4x4 numpy array representing the homogeneous
                               transformation matrix.
        sequence (str, optional): The Euler angle sequence (e.g., 'zyx', 'xyz').
                                  Defaults to 'zyx'.
        in_deg (bool, optional): If True, Euler angles are returned in degrees.
                                  Otherwise, in radians. Defaults to False.

    Returns:
        np.ndarray: A 1x3 numpy array containing the Euler angles (e.g., [yaw, pitch, roll] for 'zyx').
    """
    if T_mat.shape != (4, 4):
        raise ValueError("Input matrix must be a 4x4 homogeneous transformation matrix.")

    # Extract the 3x3 rotation matrix
    rot_mat = T_mat[:3, :3]

    # Create a Rotation object from the rotation matrix
    r = Rotation.from_matrix(rot_mat)

    # Convert to Euler angles
    euler_angles = r.as_euler(sequence, degrees=in_deg)

    return euler_angles

@dataclass
class TeleButtonData: 
    # controller tracking
    # https://docs.vuer.ai/en/latest/examples/20_motion_controllers.html
    # https://immersive-web.github.io/webxr-gamepads-module/
    left_ctrl_trigger: bool = False        # True if trigger is actively pressed
    left_ctrl_triggerValue: float = 10.0   # float (10.0 → 0.0) trigger pull depth, 0.0 means fully pressed (for align with hand pinch value's logic)
    left_ctrl_squeeze: bool = False        # True if grip button is pressed
    left_ctrl_squeezeValue: float = 0.0    # (0.0 → 1.0) grip pull depth, 0.0 means no press
    left_ctrl_aButton: bool = False        # True if A(X) button is pressed
    left_ctrl_bButton: bool = False        # True if B(Y) button is pressed
    left_ctrl_thumbstick: bool = False     # True if thumbstick button is pressed
    left_ctrl_thumbstickValue: np.ndarray = field(default_factory=lambda: np.zeros(2)) # 2D vector (x, y), normalized
    """ thumbstickValue explanation:
                    front (0,-1)
                       ^
                       |
      left (-1,0) < —— o —— > right (1,0)      and 'o' is at (0, 0)
                       |
                       v
                    back (0,1)
    """
    right_ctrl_trigger: bool = False       # True if trigger is actively pressed
    right_ctrl_triggerValue: float = 10.0  # float (10.0 → 0.0) trigger pull depth, 0.0 means fully pressed (for align  with hand pinch value's logic)
    right_ctrl_squeeze: bool = False       # True if grip button is pressed
    right_ctrl_squeezeValue: float = 0.0   # (0.0 → 1.0) grip pull depth, 0.0 means no press
    right_ctrl_aButton: bool = False       # True if A button is pressed
    right_ctrl_bButton: bool = False       # True if B button is pressed
    right_ctrl_thumbstick: bool = False    # True if thumbstick button is pressed
    right_ctrl_thumbstickValue: np.ndarray = field(default_factory=lambda: np.zeros(2)) # 2D vector (x, y), normalized

class T1TeleopWrapper:
    """
    A simplified wrapper for TeleVuer tailored for the T1 robot.
    Handles VR data acquisition and coordinate frame transformations with a calibration step.

    It initializes the TeleVuer instance with the specified parameters and provides a method to get motion state data.

    :param use_hand_tracking: bool, whether to use hand tracking or controller tracking.
    :param binocular: bool, whether the application is binocular (stereoscopic) or monocular.
    :param img_shape: tuple, shape of the head image (height, width).
    :param display_fps: float, target frames per second for display updates (default: 30.0).

    :param display_mode: str, controls the VR viewing mode. Options are "immersive", "pass-through", and "ego".
    :param zmq: bool, whether to use ZMQ for image transmission.
    :param webrtc: bool, whether to use webrtc for real-time communication.
    :param webrtc_url: str, URL for the webrtc offer. must be provided if webrtc is True.
    :param cert_file: str, path to the SSL certificate file.
    :param key_file: str, path to the SSL key file.

    Note:

    - display_mode controls what the VR headset displays:
        * "immersive": fully immersive mode; VR shows the robot's first-person view (zmq or webrtc must be enabled).
        * "pass-through": VR shows the real world through the VR headset cameras; no image from zmq or webrtc is displayed (even if enabled).
        * "ego": a small window in the center shows the robot's first-person view, while the surrounding area shows the real world.
    
    - Only one image mode is active at a time.
    - Image transmission to VR occurs only if display_mode is "immersive" or "ego" and the corresponding zmq or webrtc option is enabled.
    - If zmq and webrtc simultaneously enabled, webrtc will be prioritized.

    --------------              -------------------           --------------       -----------------                     -------
     display_mode       |        display behavior         |    image to VR     |      image source        |               Notes
    --------------              -------------------           --------------       -----------------                     ------- 
       immersive        |   fully immersive view (robot)  |     Yes (full)     |     zmq or webrtc        |   if both enabled, webrtc prioritized
    --------------              -------------------           --------------       -----------------                     -------
     pass-through       |       Real world view (VR)      |         No         |          N/A             |  even if image source enabled, don't display
    --------------              -------------------           --------------       -----------------                     -------
          ego           |      ego view (robot + VR)      |    Yes (small)     |     zmq or webrtc        |   if both enabled, webrtc prioritized
    --------------              -------------------           --------------       -----------------                     -------
    """
    def __init__(self, use_hand_tracking: bool=False, binocular: bool=False, 
                img_shape: tuple=(480, 1280), display_fps: float=30.0,
                display_mode: Literal["immersive", "pass-through", "ego"]="immersive", 
                zmq: bool=False, webrtc: bool=False, webrtc_url: str=None, 
                cert_file: str=None, key_file: str=None, return_hand_rot_data: bool=False):

        self.tvuer = TeleVuer(
            use_hand_tracking=use_hand_tracking, binocular=binocular, img_shape=img_shape,
            display_fps=display_fps, zmq=zmq, webrtc=webrtc, webrtc_url=webrtc_url, 
            cert_file=cert_file, key_file=key_file
        )

        self.is_calibrated = False
        self.T_robotbase_vrworld_L = pin.SE3.Identity()
        self.T_robotbase_vrworld_R = pin.SE3.Identity()
        
        # NEW: Neutral pose offset and arm scaling (replaces hardcoded transforms)
        self.vrcontroller_robotee_L = pin.SE3.Identity()
        self.vrcontroller_robotee_R = pin.SE3.Identity()
        self.arm_span_scale = 1.0
        
        # These are now calculated dynamically during T-pose calibration
        # No need for hardcoded T_robotee_vrcontroller transforms

    def has_valid_poses(self):
        """Checks if the VR controllers are tracked and providing non-zero poses."""
        raw_pose_L_xr = self.tvuer.left_arm_pose
        raw_pose_R_xr = self.tvuer.right_arm_pose
        # Check if the rotation matrices are not identity (a common default value)
        return (not np.allclose(raw_pose_L_xr[:3,:3], np.eye(3)) and \
               not np.allclose(raw_pose_R_xr[:3,:3], np.eye(3))) and \
               (not np.allclose(raw_pose_L_xr[:3,:3], np.zeros((3,3))) and \
                not np.allclose(raw_pose_R_xr[:3,:3], np.zeros((3,3))))

    def get_motion_state_data(self):
        if not self.has_valid_poses():
            return None, None, None

        R_offset_left  = pin.rpy.rpyToMatrix(0, 0, 0)# -np.pi/2.0)
        R_offset_right = pin.rpy.rpyToMatrix(0, 0, 0)# np.pi/2.0)

        offset_left = pin.SE3(R_offset_left, np.zeros(3))
        offset_right = pin.SE3(R_offset_right, np.zeros(3))

        XR_world_head = self.tvuer.head_pose
        XR_world_vrcontroller_L = self.tvuer.left_arm_pose 
        XR_world_vrcontroller_R = self.tvuer.right_arm_pose

        ROBOT_world_head = T_ROBOT_OPENXR @ XR_world_head @ T_OPENXR_ROBOT
        ROBOT_world_vrcontroller_L = T_ROBOT_OPENXR @ XR_world_vrcontroller_L @ T_OPENXR_ROBOT
        ROBOT_world_vrcontroller_R = T_ROBOT_OPENXR @ XR_world_vrcontroller_R @ T_OPENXR_ROBOT
        
        ROBOT_world_robotee_L = ROBOT_world_vrcontroller_L @ offset_left
        ROBOT_world_robotee_R = ROBOT_world_vrcontroller_R @ offset_right

        # Transfer from WORLD to HEAD coordinate (translation adjustment only)
        ROBOT_head_robotee_L = ROBOT_world_robotee_L.copy()
        ROBOT_head_robotee_R = ROBOT_world_robotee_R.copy()

        # original
        # ROBOT_head_robotee_L[0:3, 3] = ROBOT_world_robotee_L[0:3, 3] - ROBOT_world_head[0:3, 3]
        # ROBOT_head_robotee_R[0:3, 3] = ROBOT_world_robotee_R[0:3, 3] - ROBOT_world_head[0:3, 3]

        # print("ROBOT_world_head: ", ROBOT_world_head)
        # print(type(ROBOT_world_head))
        # print("ROBOT_world_robotee_L: ", ROBOT_world_robotee_L)
        # print(type(ROBOT_world_robotee_L))
        # print("ROBOT_world_robotee_R: ", ROBOT_world_robotee_R)
        # print(type(ROBOT_world_robotee_R))

        ### For new version of vuer have to check to rebug headset position input
        # if XR_world_head is None or XR_world_head[3,3] != 1:
        #     print("XR_world_head: ", XR_world_head)
        #     return None, None, None

        ROBOT_head_robotee_L = np.linalg.inv(ROBOT_world_head) @ ROBOT_world_robotee_L
        ROBOT_head_robotee_R = np.linalg.inv(ROBOT_world_head) @ ROBOT_world_robotee_R

        ROBOT_trunk_robotee_L = ROBOT_head_robotee_L.copy()
        ROBOT_trunk_robotee_R = ROBOT_head_robotee_R.copy()
        '''
        TF Trunk to H2 (head pitch joint)
        - Translation: [0.062, 0.000, 0.305]
        - Rotation: in Quaternion [0.000, -0.000, 0.000, 1.000]
        - Rotation: in RPY (radian) [0.000, -0.000, 0.000]
        - Rotation: in RPY (degree) [0.000, -0.003, 0.000]

        # Before 5 Mar 2026: we add 5 more cm to center of head and 15cm to account for vr headset thickness
        # 6 Mar 2026: YC tuned to +10 cm instead in x. 
        '''
        ROBOT_trunk_robotee_L[2,3] += 0.355
        ROBOT_trunk_robotee_R[2,3] += 0.355
       
        ROBOT_trunk_robotee_L[0,3] += 0.10 # 0.2
        ROBOT_trunk_robotee_R[0,3] += 0.10 # 0.2

        target_L = pin.SE3(ROBOT_trunk_robotee_L)
        target_R = pin.SE3(ROBOT_trunk_robotee_R)
        robot_head_pose = pin.SE3(ROBOT_world_head)

        return (robot_head_pose, target_L, target_R)

    def get_button_state_data(self):
        return TeleButtonData(
            left_ctrl_trigger=self.tvuer.left_ctrl_trigger,
            left_ctrl_triggerValue=10.0 - self.tvuer.left_ctrl_triggerValue * 10,
            left_ctrl_squeeze=self.tvuer.left_ctrl_squeeze,
            left_ctrl_squeezeValue=self.tvuer.left_ctrl_squeezeValue,
            left_ctrl_aButton=self.tvuer.left_ctrl_aButton,
            left_ctrl_bButton=self.tvuer.left_ctrl_bButton,
            left_ctrl_thumbstick=self.tvuer.left_ctrl_thumbstick,
            left_ctrl_thumbstickValue=self.tvuer.left_ctrl_thumbstickValue,
            right_ctrl_trigger=self.tvuer.right_ctrl_trigger,
            right_ctrl_triggerValue=10.0 - self.tvuer.right_ctrl_triggerValue * 10,
            right_ctrl_squeeze=self.tvuer.right_ctrl_squeeze,
            right_ctrl_squeezeValue=self.tvuer.right_ctrl_squeezeValue,
            right_ctrl_aButton=self.tvuer.right_ctrl_aButton,
            right_ctrl_bButton=self.tvuer.right_ctrl_bButton,
            right_ctrl_thumbstick=self.tvuer.right_ctrl_thumbstick,
            right_ctrl_thumbstickValue=self.tvuer.right_ctrl_thumbstickValue,
        )

    def calibrate(self, T_robotbase_robotee_initial_L: pin.SE3, T_robotbase_robotee_initial_R: pin.SE3):
        """
        Enhanced calibration with T-pose neutral pose offset estimation.

        Args:
            T_robotbase_robotee_initial_L: The SE3 pose of the left robot EE in its base frame.
            T_robotbase_robotee_initial_R: The SE3 pose of the right robot EE in its base frame.
        """
        print("INSIDE ENHANCED CALIBRATION")

        # Step 1: T-pose calibration for neutral pose offset
        t_pose_data = self.collect_t_pose_reference(T_robotbase_robotee_initial_L, T_robotbase_robotee_initial_R)
        
        if not t_pose_data:
            print("T-pose calibration failed.")
            return False
            
        self.vrcontroller_robotee_L, self.vrcontroller_robotee_R = self._estimate_neutral_pose_offset(t_pose_data)

        # Step 2: Regular pose calibration
        print("T-pose calibration complete!")
        print("Now match the robot's neutral pose for final calibration...")
        print("Calibration will start automatically when your pose is stable.")
        
        # Wait for user to match neutral pose
        stable_neutral_poses = self._wait_for_stable_vr_poses(timeout=30.0)
        
        if not stable_neutral_poses:
            print("Neutral pose calibration failed.")
            return False
        
        print("Neutral pose detected as stable. Completing calibration...")

        # Get current raw controller poses from TeleVuer
        raw_pose_L_xr = self.tvuer.left_arm_pose
        raw_pose_R_xr = self.tvuer.right_arm_pose

        if not self.has_valid_poses():
            print("Calibration failed: Invalid or default controller poses received.")
            return False

        # Convert raw poses from OpenXR basis to Robot basis
        T_vrworld_vrcontroller_L_robot_basis = pin.SE3(T_ROBOT_OPENXR @ raw_pose_L_xr @ T_OPENXR_ROBOT)
        T_vrworld_vrcontroller_R_robot_basis = pin.SE3(T_ROBOT_OPENXR @ raw_pose_R_xr @ T_OPENXR_ROBOT)
        
        print("Raw Left Pose in Robot Basis:", T_vrworld_vrcontroller_L_robot_basis)
        print("Raw Right Pose in Robot Basis:", T_vrworld_vrcontroller_R_robot_basis)

        # Apply neutral pose offset before calibration
        T_vrworld_robotee_L = T_vrworld_vrcontroller_L_robot_basis * self.vrcontroller_robotee_L
        T_vrworld_robotee_R = T_vrworld_vrcontroller_R_robot_basis * self.vrcontroller_robotee_R 

        # Calculate the required transformation with offset applied
        # No need for hardcoded T_robotee_vrcontroller transforms - offset handles this
        self.T_robotbase_vrworld_L = T_robotbase_robotee_initial_L * T_vrworld_robotee_L.inverse()
        self.T_robotbase_vrworld_R = T_robotbase_robotee_initial_R * T_vrworld_robotee_R.inverse()

        # self.T_robotbase_vrworld_L = T_vrworld_vrcontroller_L_offset 
        # self.T_robotbase_vrworld_R = T_vrworld_vrcontroller_R_offset 

        print(f"Robot Base to VR World Transform (Left): {self.T_robotbase_vrworld_L}")
        print(f"Robot Base to VR World Transform (Right): {self.T_robotbase_vrworld_R}")
        
        print("Enhanced calibration successful. VR poses will now be transformed with neutral pose offset and arm scaling.")
        print("=====================================================================================")

        self.is_calibrated = True
        return True

    def get_target_poses(self):
        """
        Gets the current controller poses and transforms them into the robot's base frame.

        Returns:
            A tuple (left_target_pose, right_target_pose) of pin.SE3 objects,
            or (None, None) if not calibrated or poses are invalid.
        """
        if not self.is_calibrated:
            print("Warning: get_target_poses() called before calibration.")
            return None, None
            
        # 1. Get raw poses and check if they are valid
        if not self.has_valid_poses():
            return None, None
        
        raw_pose_L_xr = self.tvuer.left_arm_pose
        raw_pose_R_xr = self.tvuer.right_arm_pose

        # print("Raw Left Pose:", raw_pose_L_xr, "\nRaw Right Pose:", raw_pose_R_xr)

        # 2. Convert to Robot Basis
        T_vrworld_vrcontroller_L_robot_basis = pin.SE3(T_ROBOT_OPENXR @ raw_pose_L_xr @ T_OPENXR_ROBOT)
        T_vrworld_vrcontroller_R_robot_basis = pin.SE3(T_ROBOT_OPENXR @ raw_pose_R_xr @ T_OPENXR_ROBOT)

        # 3. Apply neutral pose offset
        # T_vrworld_vrcontroller_L_offset = self.neutral_pose_offset_L * T_vrworld_vrcontroller_L_robot_basis
        # T_vrworld_vrcontroller_R_offset = self.neutral_pose_offset_R * T_vrworld_vrcontroller_R_robot_basis

        # 4. Apply arm scaling
        T_vrworld_vrcontroller_L_scaled = self._apply_arm_scaling(T_vrworld_vrcontroller_L_robot_basis, self.arm_span_scale)
        T_vrworld_vrcontroller_R_scaled = self._apply_arm_scaling(T_vrworld_vrcontroller_R_robot_basis, self.arm_span_scale)

        # 5. Apply calibration transformation
        # The neutral pose offset already handles the controller-to-end-effector alignment
        target_L = self.T_robotbase_vrworld_L * T_vrworld_vrcontroller_L_scaled * self.vrcontroller_robotee_L 
        target_R = self.T_robotbase_vrworld_R * T_vrworld_vrcontroller_R_scaled * self.vrcontroller_robotee_R

        return target_L, target_R

    def _estimate_neutral_pose_offset(self, t_pose_data):
        """
        Estimate neutral pose offset using T-pose as reference.
        Calculates separate translation and rotation offsets for left and right arms.
        
        Args:
            t_pose_data: Dictionary with human and robot T-pose positions
                        {'human_left': SE3, 'human_right': SE3, 
                         'robot_left': SE3, 'robot_right': SE3}
        
        Returns:
            tuple: (left_offset, right_offset) - pin.SE3 transformations for each arm
        """
        # Extract positions from T-pose data
        human_left_pos = t_pose_data['human_left'].translation
        human_right_pos = t_pose_data['human_right'].translation
        robot_left_pos = t_pose_data['robot_left'].translation
        robot_right_pos = t_pose_data['robot_right'].translation
        
        # Calculate centers (shoulder/torso reference)
        # human_center = (human_left_pos + human_right_pos) / 2
        # robot_center = (robot_left_pos + robot_right_pos) / 2
        
        # Calculate arm spans for scaling
        human_arm_span = np.linalg.norm(human_left_pos - human_right_pos)
        robot_arm_span = np.linalg.norm(robot_left_pos - robot_right_pos)
        
        # Translation offset (center alignment) - same for both arms
        # translation_offset = robot_center - human_center
        # translation_offset = robot_center - robot_center
        
        # Store arm span ratio for scaling
        # self.arm_span_scale = robot_arm_span / human_arm_span if human_arm_span > 0 else 1.0
        self.arm_span_scale = 1.0

        # Calculate separate rotation offsets for each arm
        human_left_rot = t_pose_data['human_left'].rotation
        human_right_rot = t_pose_data['human_right'].rotation
        robot_left_rot = t_pose_data['robot_left'].rotation
        robot_right_rot = t_pose_data['robot_right'].rotation
        
        # Calculate rotation offset for each arm independently
        # R_offset_left = robot_left_rot @ human_left_rot.T
        # R_offset_right = robot_right_rot @ human_right_rot.T
        
        # vr_controller_robot_ee
        R_offset_left = human_left_rot.T @ robot_left_rot 
        R_offset_right = human_right_rot.T @ robot_right_rot 

        # R_offset_left = pin.rpy.rpyToMatrix(0,0,-np.pi/2.0)
        # R_offset_right = pin.rpy.rpyToMatrix(0,0,np.pi/2.0)

        # print(f"T-pose center offset: {translation_offset}")
        print(f"Arm span scale factor: {self.arm_span_scale}")
        print(f"Separate rotation offsets calculated for left and right arms")
        print(f"R_offset_left: {R_offset_left}")
        print(f"R_offset_right: {R_offset_right}")
        
        # Return separate offsets for each arm
        offset_left = pin.SE3(R_offset_left, np.zeros(3))
        offset_right = pin.SE3(R_offset_right, np.zeros(3))
        
        return offset_left, offset_right

    def _wait_for_stable_vr_poses(self, timeout=30.0, stability_duration=2.0):
        """
        Wait for VR poses to be stable in T-pose configuration.
        
        Args:
            timeout: Maximum time to wait for stability (seconds)
            stability_duration: How long pose must be stable (seconds)
        
        Returns:
            bool: True if poses became stable, False if timeout
        """
        import time
        
        print("Waiting for stable T-pose...")
        
        pose_history = []
        stable_start_time = None
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.has_valid_poses():
                time.sleep(0.1)
                continue
                
            # Get current poses
            current_L = self.tvuer.left_arm_pose[:3, 3]
            current_R = self.tvuer.right_arm_pose[:3, 3]
            current_pose = np.concatenate([current_L, current_R])
            
            pose_history.append(current_pose)
            
            # Keep only recent history (2 seconds at 10Hz = 20 samples)
            if len(pose_history) > 20:
                pose_history.pop(0)
            
            # Check if T-pose is stable
            if len(pose_history) >= 20:
                pose_array = np.array(pose_history)
                pose_std = np.std(pose_array, axis=0)
                
                # T-pose stability: arms should be relatively still
                if np.all(pose_std < 0.05):  # 2cm stability threshold
                    if stable_start_time is None:
                        stable_start_time = time.time()
                        print("T-pose becoming stable... hold position...")
                    elif time.time() - stable_start_time >= stability_duration:
                        print("T-pose is stable!")
                        return True
                else:
                    stable_start_time = None
            
            time.sleep(0.1)  # 10Hz checking
        
        return False

    def collect_t_pose_reference(self, robot_left_t_pose, robot_right_t_pose):
        """
        Automated T-pose reference collection with stability detection.
        
        Args:
            robot_left_t_pose: Robot left arm T-pose SE3
            robot_right_t_pose: Robot right arm T-pose SE3
        
        Returns:
            Dictionary with T-pose reference data or None if failed
        """
        print("=== T-POSE CALIBRATION ===")
        print("Robot is now in T-pose position.")
        print("Please extend your arms horizontally to match the robot's T-pose...")
        print("Calibration will start automatically when your pose is stable.")
        
        # Wait for user VR pose to stabilize in T-pose
        stable_vr_poses = self._wait_for_stable_vr_poses(timeout=30.0)
        
        if stable_vr_poses:
            print("T-pose detected as stable. Capturing calibration data...")
            raw_pose_L_xr = self.tvuer.left_arm_pose
            raw_pose_R_xr = self.tvuer.right_arm_pose
            raw_pose_HEAD_xr = self.tvuer.head_pose 
            
            # Convert to Robot Basis
            human_left_t = pin.SE3(T_ROBOT_OPENXR @ raw_pose_L_xr @ T_OPENXR_ROBOT)
            human_right_t = pin.SE3(T_ROBOT_OPENXR @ raw_pose_R_xr @ T_OPENXR_ROBOT)
            human_head_t = pin.SE3(T_ROBOT_OPENXR @ raw_pose_HEAD_xr @ T_OPENXR_ROBOT) 

            return {
                'human_left': human_left_t,
                'human_right': human_right_t,
                'robot_left': robot_left_t_pose,
                'robot_right': robot_right_t_pose
            }
        else:
            print("Timeout waiting for stable T-pose. Please try again.")
            return None

    def _apply_arm_scaling(self, pose, scale_factor):
        """
        Apply arm scaling to a pose while preserving orientation.
        
        Args:
            pose: pin.SE3 pose to scale
            scale_factor: Scaling factor for translation
        
        Returns:
            pin.SE3: Scaled pose
        """
        scaled_translation = pose.translation * scale_factor
        return pin.SE3(pose.rotation, scaled_translation)

    def render_to_xr(self, img, rgb_img_encoding):
        self.tvuer.render_to_xr(img, rgb_img_encoding)

    def update_popup_text(self, text):
        self.tvuer.update_popup_text(text)
    
    def close(self):
        self.tvuer.close()