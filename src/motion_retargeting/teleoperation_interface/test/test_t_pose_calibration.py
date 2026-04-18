#!/usr/bin/env python3
"""
Test script for T-pose neutral pose offset estimation.
This demonstrates the new functionality without requiring full VR setup.
"""

import numpy as np
import sys
import os

# Add the t1_teleop_ik package to the path
sys.path.append('/home/sde_ws/src/motion_retargeting/ik_solvers/casadi_optimal_control_ik')

def test_t_pose_offset_estimation():
    """
    Test the T-pose offset estimation with mock data.
    """
    print("=== T-POSE OFFSET ESTIMATION TEST ===")
    
    # Mock T-pose data (simulating human and robot poses)
    mock_t_pose_data = {
        'human_left': MockSE3(translation=np.array([-0.7, 0.0, 1.5])),    # Human left arm extended
        'human_right': MockSE3(translation=np.array([0.7, 0.0, 1.5])),    # Human right arm extended  
        'robot_left': MockSE3(translation=np.array([-0.5, 0.0, 1.2])),    # Robot left arm extended
        'robot_right': MockSE3(translation=np.array([0.5, 0.0, 1.2]))     # Robot right arm extended
    }
    
    # Create a mock wrapper to test the estimation
    wrapper = MockT1TeleopWrapper()
    
    # Test the offset estimation
    offset = wrapper._estimate_neutral_pose_offset(mock_t_pose_data)
    
    print("Estimated neutral pose offset:", offset.translation)
    print("Estimated arm span scale:", wrapper.arm_span_scale)
    
    # Test arm scaling
    test_pose = MockSE3(translation=np.array([0.3, 0.2, 1.0]))
    scaled_pose = wrapper._apply_arm_scaling(test_pose, wrapper.arm_span_scale)
    
    print("Original pose:", test_pose.translation)
    print("Scaled pose:", scaled_pose.translation)
    
    print("=== TEST COMPLETED ===")

class MockSE3:
    """Mock SE3 class for testing without pinocchio dependency."""
    def __init__(self, rotation=None, translation=None):
        self.rotation = rotation if rotation is not None else np.eye(3)
        self.translation = translation if translation is not None else np.zeros(3)

class MockT1TeleopWrapper:
    """Mock wrapper class for testing T-pose functionality."""
    def __init__(self):
        self.neutral_pose_offset = MockSE3()
        self.arm_span_scale = 1.0
    
    def _estimate_neutral_pose_offset(self, t_pose_data):
        """
        Estimate neutral pose offset using T-pose as reference.
        """
        # Extract positions from T-pose data
        human_left_pos = t_pose_data['human_left'].translation
        human_right_pos = t_pose_data['human_right'].translation
        robot_left_pos = t_pose_data['robot_left'].translation
        robot_right_pos = t_pose_data['robot_right'].translation
        
        # Calculate centers (shoulder/torso reference)
        human_center = (human_left_pos + human_right_pos) / 2
        robot_center = (robot_left_pos + robot_right_pos) / 2
        
        # Calculate arm spans for scaling
        human_arm_span = np.linalg.norm(human_left_pos - human_right_pos)
        robot_arm_span = np.linalg.norm(robot_left_pos - robot_right_pos)
        
        # Simple translation offset (center alignment)
        translation_offset = robot_center - human_center
        
        # Store arm span ratio for scaling
        self.arm_span_scale = robot_arm_span / human_arm_span if human_arm_span > 0 else 1.0
        
        print("Human center:", human_center)
        print("Robot center:", robot_center)
        print("Human arm span: {:.3f}m".format(human_arm_span))
        print("Robot arm span: {:.3f}m".format(robot_arm_span))
        print("T-pose center offset:", translation_offset)
        print("Arm span scale factor: {:.3f}".format(self.arm_span_scale))
        
        return MockSE3(translation=translation_offset)
    
    def _apply_arm_scaling(self, pose, scale_factor):
        """
        Apply arm scaling to a pose while preserving orientation.
        """
        scaled_translation = pose.translation * scale_factor
        return MockSE3(rotation=pose.rotation, translation=scaled_translation)

if __name__ == "__main__":
    test_t_pose_offset_estimation()