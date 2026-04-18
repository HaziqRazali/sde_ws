# -----------------------------------------------------------
# File: ik_config_loader.py
# Description: Configuration loader for IK Solver using YAML
#
# Author: Hari Prasanth
# Date Created: 25 July 2025

import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from typing import Dict, List, Any, Optional

class IKConfigLoader:
    """Loads and validates configuration from YAML file for T1 IK Solver."""
    
    def __init__(self, config_file_path: str = None):
        """
        Initialize the config loader.
        
        Args:
            config_file_path: Path to YAML config file. If None, uses default location.
        """
        if config_file_path is None:
            # Default config location in package
            try:
                package_share = get_package_share_directory('casadi_optimal_control_ik')
                config_file_path = os.path.join(package_share, 'config', 'ik_config.yaml')
            except Exception:
                raise FileNotFoundError("Could not find default config file. Please specify config_file_path.")
        
        self.config_file_path = config_file_path
        self.config = self._load_config()
        self._validate_config()
    
    def _load_config(self) -> Dict[str, Any]:
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
    
    def _validate_config(self):
        """Validate that required configuration sections exist."""
        required_sections = ['robot_model', 'joints', 'optimization', 'collision', 'ros2']
        for section in required_sections:
            if section not in self.config:
                raise ValueError(f"Missing required config section: {section}")
    
    # Robot Model Properties
    @property
    def description_package_name(self) -> str:
        return self.config['robot_model']['description_package_name']
    
    @property
    def urdf_path(self) -> str:
        return self.config['robot_model']['urdf_path']
    
    @property
    def left_hand_joint(self) -> str:
        return self.config['robot_model']['end_effectors']['left']['hand_joint']
    
    @property
    def right_hand_joint(self) -> str:
        return self.config['robot_model']['end_effectors']['right']['hand_joint']
    
    @property
    def left_ee_frame_name(self) -> str:
        return self.config['robot_model']['end_effectors']['left']['ee_frame_name']
    
    @property
    def right_ee_frame_name(self) -> str:
        return self.config['robot_model']['end_effectors']['right']['ee_frame_name']
    
    @property
    def left_ee_translation_offset(self) -> np.ndarray:
        offset = self.config['robot_model']['end_effectors']['left']['ee_translation_offset']
        return np.array(offset)
    
    @property
    def right_ee_translation_offset(self) -> np.ndarray:
        offset = self.config['robot_model']['end_effectors']['right']['ee_translation_offset']
        return np.array(offset)

    @property
    def left_ee_rotation_offset(self) -> np.ndarray:
        rot_matrix = self.config['robot_model']['end_effectors']['left']['ee_rotation_offset']
        return np.array(rot_matrix)

    @property
    def right_ee_rotation_offset(self) -> np.ndarray:
        rot_matrix = self.config['robot_model']['end_effectors']['right']['ee_rotation_offset']
        return np.array(rot_matrix)

    # Joint Configuration
    @property
    def reference_configuration(self) -> np.ndarray:
        rot_matrix = self.config['joints']['reference_configuration']
        return np.array(rot_matrix)

    @property
    def reduced_reference_configuration(self) -> np.ndarray:
        rot_matrix = self.config['joints']['reduced_reference_configuration']
        return np.array(rot_matrix)

    @property
    def joints_to_lock(self) -> List[str]:
        return self.config['joints']['joints_to_lock']

    @property
    def head_control_joints(self) -> List[str]:
        return self.config['joints']['head_control_joints']

    @property
    def regularization_q_offset(self) -> np.ndarray:
        return np.array(self.config['joints']['regularization_q_offset'])
    
    # Optimization Parameters
    @property
    def weight_translation(self) -> float:
        return self.config['optimization']['weights']['translation']
    
    @property
    def weight_rotation(self) -> float:
        return self.config['optimization']['weights']['rotation']
    
    @property
    def weight_regularization(self) -> float:
        return self.config['optimization']['weights']['regularization']
    
    @property
    def weight_smoothness(self) -> float:
        return self.config['optimization']['weights']['smoothness']

    @property
    def weight_collision(self) -> float:
        return self.config['optimization']['weights']['collision']
            
    @property
    def wrist_yaw_joints_name(self) -> List[int]:
        return self.config['optimization']['penalty']['wrist_yaw_joints_name']

    @property
    def penalty_wrist_yaw(self) -> float:
        return self.config['optimization']['penalty']['wrist_yaw']

    @property
    def wrist_pitch_joints_name(self) -> List[int]:
        return self.config['optimization']['penalty']['wrist_pitch_joints_name']

    @property
    def penalty_wrist_pitch(self) -> float:
        return self.config['optimization']['penalty']['wrist_pitch']
    
    @property
    def shoulder_roll_joints_name(self) -> List[int]:
        return self.config['optimization']['penalty']['shoulder_roll_joints_name']

    @property
    def penalty_shoulder_roll(self) -> float:
        return self.config['optimization']['penalty']['shoulder_roll']

    @property
    def solver_print_level(self) -> int:
        return self.config['optimization']['solver']['print_level']
    
    @property
    def solver_print_time(self) -> bool:
        return self.config['optimization']['solver']['print_time']
    
    # Filtering
    @property
    def smoothing_filter_weights(self) -> np.ndarray:
        return np.array(self.config['filtering']['smoothing_weights'])
    
    # Collision configurations for optimization
    @property
    def link_collision_safety_dist(self) -> float:
        return self.config['collision']['link_safety_dist']

    @property
    def link_collision_pairs(self) -> np.ndarray:
        return self.config['collision']['link_collision_pairs']

    # ROS 2 Configuration  
    @property
    def node_name(self) -> str:
        return self.config['ros2']['node_name']
    
    @property
    def joint_state_topic(self) -> str:
        return self.config['ros2']['topics']['joint_states']
    
    @property
    def target_pose_topic(self) -> str:
        return self.config['ros2']['topics']['target_poses']
    
    @property
    def num_target_poses(self) -> int:
        return self.config['ros2']['topics']['num_target_poses']

    @property
    def type_target_poses(self) -> np.ndarray:
        return np.array(self.config['ros2']['topics']['type_target_poses'])
    
    @property
    def publish_rate_hz(self) -> float:
        return self.config['ros2']['publish_rate_hz']
    
    @property
    def qos_depth(self) -> int:
        return self.config['ros2']['qos_depth']
    
    @property
    def robot_root_frame(self) -> str:
        return self.config['ros2']['frames']['robot_root']
    
    @property
    def left_target_tf_frame(self) -> str:
        return self.config['ros2']['frames']['left_target']
    
    @property
    def right_target_tf_frame(self) -> str:
        return self.config['ros2']['frames']['right_target']
    
    @property
    def left_real_ee_tf_frame(self) -> str:
        return self.config['ros2']['frames']['left_hand_fname']
    
    @property
    def right_real_ee_tf_frame(self) -> str:
        return self.config['ros2']['frames']['right_hand_fname']

    @property
    def ee_frame_listen_rate(self) -> float:
        return self.config['ros2']['frames']['ee_frame_listen_rate']
    

    # Optional Advanced Parameters
    def get_workspace_limits(self) -> Optional[Dict[str, float]]:
        """Get workspace limits if defined in config."""
        if 'safety' in self.config and 'workspace_limits' in self.config['safety']:
            return self.config['safety']['workspace_limits']
        return None
    
    def get_max_joint_delta(self) -> Optional[float]:
        """Get maximum joint delta per iteration if defined."""
        if 'safety' in self.config and 'max_joint_delta' in self.config['safety']:
            return self.config['safety']['max_joint_delta']
        return None

    @property
    def debug_variable(self) -> bool:
        return self.config['debug']

    @property
    def verbose_variable(self) -> bool:
        return self.config['verbose']

# Modified IKConfig class to use YAML configuration
class IKConfig:
    """
    Configuration parameters for the T1 IK Solver - now loaded from YAML.
    This class maintains the same interface as before but loads from config file.
    """
    
    def __init__(self, config_file_path: str = None):
        """
        Initialize configuration from YAML file.
        
        Args:
            config_file_path: Path to YAML config file. If None, uses default.
        """
        self.loader = IKConfigLoader(config_file_path)
        
        # Load all configuration values
        self.DESCRIPTION_PACKAGE_NAME = self.loader.description_package_name
        self.URDF_PATH = self.loader.urdf_path
        self.L_HAND_JOINT = self.loader.left_hand_joint
        self.R_HAND_JOINT = self.loader.right_hand_joint
        self.L_EE_FRAME_NAME = self.loader.left_ee_frame_name
        self.R_EE_FRAME_NAME = self.loader.right_ee_frame_name
        
        # Calculate translation offsets 
        # [x, y, z]
        self.LEFT_EE_TRANS_OFFSET = self.loader.left_ee_translation_offset
        self.RIGHT_EE_TRANS_OFFSET = self.loader.right_ee_translation_offset
        self.LEFT_EE_ROT_OFFSET = self.loader.left_ee_rotation_offset
        self.RIGHT_EE_ROT_OFFSET = self.loader.right_ee_rotation_offset

        self.REFERENCE_CONFIGURATION = self.loader.reference_configuration
        self.REDUCED_REFERENCE_CONFIGURATION = self.loader.reduced_reference_configuration
        self.JOINTS_TO_LOCK = self.loader.joints_to_lock
        self.HEAD_CONTROL_JOINTS = self.loader.head_control_joints
        self.WRIST_YAW_JOINTS_NAME = self.loader.wrist_yaw_joints_name
        self.WRIST_PITCH_JOINTS_NAME = self.loader.wrist_pitch_joints_name
        self.SHOULDER_ROLL_JOINTS_NAME = self.loader.shoulder_roll_joints_name
        self.REGULARIZATION_Q_OFFSET = self.loader.regularization_q_offset
        
        # Optimization weights
        self.WEIGHT_TRANSLATION = self.loader.weight_translation
        self.WEIGHT_ROTATION = self.loader.weight_rotation
        self.WEIGHT_REGULARIZATION = self.loader.weight_regularization
        self.WEIGHT_SMOOTHNESS = self.loader.weight_smoothness
        self.WEIGHT_COLLISION = self.loader.weight_collision
        self.WEIGHT_WRIST_YAW = self.loader.penalty_wrist_yaw
        self.WEIGHT_WRIST_PITCH = self.loader.penalty_wrist_pitch
        self.WEIGHT_SHOULDER_ROLL = self.loader.penalty_shoulder_roll
        
        # Smoothing filter
        self.SMOOTHING_FILTER_WEIGHTS = self.loader.smoothing_filter_weights
        
        # Collision configurations for optimization
        self.LINK_COLLISION_SAFETY_DIST = self.loader.link_collision_safety_dist
        self.LINK_COLLISION_PAIRS = self.loader.link_collision_pairs

        # ROS 2 parameters
        self.NODE_NAME = self.loader.node_name
        self.JOINT_STATE_TOPIC = self.loader.joint_state_topic
        self.TARGET_POSE_TOPIC = self.loader.target_pose_topic
        self.NUM_TARGET_POSES = self.loader.num_target_poses
        self.TYPE_TARGET_POSES = self.loader.type_target_poses
        self.PUBLISH_RATE_HZ = self.loader.publish_rate_hz
        self.QOS_DEPTH = self.loader.qos_depth
        self.ROBOT_ROOT_FRAME = self.loader.robot_root_frame
        self.LEFT_TARGET_TF_FRAME = self.loader.left_target_tf_frame
        self.RIGHT_TARGET_TF_FRAME = self.loader.right_target_tf_frame
        self.LEFT_EE_TF_FRAME = self.loader.left_real_ee_tf_frame
        self.RIGHT_EE_TF_FRAME = self.loader.right_real_ee_tf_frame
        self.EE_TF_SAMPLE_HZ = self.loader.ee_frame_listen_rate

        # Optional parameters
        self.workspace_limits = self.loader.get_workspace_limits()
        self.max_joint_delta = self.loader.get_max_joint_delta()

        # Debug printing parameter
        self.DEBUG = self.loader.debug_variable
        self.VERBOSE = self.loader.verbose_variable
        
        print("IK Configuration loaded successfully from YAML")
