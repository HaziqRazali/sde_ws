#!/usr/bin/env python3
"""
MHR Subscriber Node (Python 3.12)
Subscribes to keypoints, runs MHR forward pass, logs output.
Later: publish to /ik/joint_states for robot control.

# Author: Haziq
# Date Created: 30 Jan 2026
"""

import os
import re
import json
import roma
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import torch
import torch.nn as nn
import numpy as np
from scipy.spatial.transform import Rotation

coco_wholebody = {
    "num_keypoints": 133,
    "keypoint_id2name": {
        "0": "nose", "1": "left_eye", "2": "right_eye", "3": "left_ear", "4": "right_ear",
        "5": "left_shoulder", "6": "right_shoulder", "7": "left_elbow", "8": "right_elbow",
        "9": "left_wrist", "10": "right_wrist", "11": "left_hip", "12": "right_hip",
        "13": "left_knee", "14": "right_knee", "15": "left_ankle", "16": "right_ankle",
        "17": "left_big_toe", "18": "left_small_toe", "19": "left_heel", "20": "right_big_toe",
        "21": "right_small_toe", "22": "right_heel", 
        
        "23": "face-0", "24": "face-1", "25": "face-2",
        "26": "face-3", "27": "face-4", "28": "face-5", "29": "face-6", "30": "face-7", "31": "face-8",
        "32": "face-9", "33": "face-10", "34": "face-11", "35": "face-12", "36": "face-13",
        "37": "face-14", "38": "face-15", "39": "face-16", "40": "face-17", "41": "face-18",
        "42": "face-19", "43": "face-20", "44": "face-21", "45": "face-22", "46": "face-23",
        "47": "face-24", "48": "face-25", "49": "face-26", "50": "face-27", "51": "face-28",
        "52": "face-29", "53": "face-30", "54": "face-31", "55": "face-32", "56": "face-33",
        "57": "face-34", "58": "face-35", "59": "face-36", "60": "face-37", "61": "face-38",
        "62": "face-39", "63": "face-40", "64": "face-41", "65": "face-42", "66": "face-43",
        "67": "face-44", "68": "face-45", "69": "face-46", "70": "face-47", "71": "face-48",
        "72": "face-49", "73": "face-50", "74": "face-51", "75": "face-52", "76": "face-53",
        "77": "face-54", "78": "face-55", "79": "face-56", "80": "face-57", "81": "face-58",
        "82": "face-59", "83": "face-60", "84": "face-61", "85": "face-62", "86": "face-63",
        "87": "face-64", "88": "face-65", "89": "face-66", "90": "face-67", 
        
        "91": "left_hand_root", "92": "left_thumb1", "93": "left_thumb2", "94": "left_thumb3", "95": "left_thumb4",
        "96": "left_forefinger1", "97": "left_forefinger2", "98": "left_forefinger3", "99": "left_forefinger4",
        "100": "left_middle_finger1", "101": "left_middle_finger2", "102": "left_middle_finger3", "103": "left_middle_finger4",
        "104": "left_ring_finger1", "105": "left_ring_finger2", "106": "left_ring_finger3", "107": "left_ring_finger4",
        "108": "left_pinky_finger1", "109": "left_pinky_finger2", "110": "left_pinky_finger3", "111": "left_pinky_finger4",

        "112": "right_hand_root", "113": "right_thumb1", "114": "right_thumb2", "115": "right_thumb3", "116": "right_thumb4",
        "117": "right_forefinger1", "118": "right_forefinger2", "119": "right_forefinger3", "120": "right_forefinger4",
        "121": "right_middle_finger1", "122": "right_middle_finger2", "123": "right_middle_finger3", "124": "right_middle_finger4",
        "125": "right_ring_finger1", "126": "right_ring_finger2", "127": "right_ring_finger3", "128": "right_ring_finger4",
        "129": "right_pinky_finger1", "130": "right_pinky_finger2", "131": "right_pinky_finger3", "132": "right_pinky_finger4"
    },
    "keypoint_name2id": {
        name: int(id) for id, name in {
            "0": "nose", "1": "left_eye", "2": "right_eye", "3": "left_ear", "4": "right_ear",
            "5": "left_shoulder", "6": "right_shoulder", "7": "left_elbow", "8": "right_elbow",
            "9": "left_wrist", "10": "right_wrist", "11": "left_hip", "12": "right_hip",
            "13": "left_knee", "14": "right_knee", "15": "left_ankle", "16": "right_ankle",
            "17": "left_big_toe", "18": "left_small_toe", "19": "left_heel", "20": "right_big_toe",
            "21": "right_small_toe", "22": "right_heel", "23": "face-0", "24": "face-1", "25": "face-2",
            "26": "face-3", "27": "face-4", "28": "face-5", "29": "face-6", "30": "face-7", "31": "face-8",
            "32": "face-9", "33": "face-10", "34": "face-11", "35": "face-12", "36": "face-13",
            "37": "face-14", "38": "face-15", "39": "face-16", "40": "face-17", "41": "face-18",
            "42": "face-19", "43": "face-20", "44": "face-21", "45": "face-22", "46": "face-23",
            "47": "face-24", "48": "face-25", "49": "face-26", "50": "face-27", "51": "face-28",
            "52": "face-29", "53": "face-30", "54": "face-31", "55": "face-32", "56": "face-33",
            "57": "face-34", "58": "face-35", "59": "face-36", "60": "face-37", "61": "face-38",
            "62": "face-39", "63": "face-40", "64": "face-41", "65": "face-42", "66": "face-43",
            "67": "face-44", "68": "face-45", "69": "face-46", "70": "face-47", "71": "face-48",
            "72": "face-49", "73": "face-50", "74": "face-51", "75": "face-52", "76": "face-53",
            "77": "face-54", "78": "face-55", "79": "face-56", "80": "face-57", "81": "face-58",
            "82": "face-59", "83": "face-60", "84": "face-61", "85": "face-62", "86": "face-63",
            "87": "face-64", "88": "face-65", "89": "face-66", "90": "face-67", "91": "left_hand_root",
            "92": "left_thumb1", "93": "left_thumb2", "94": "left_thumb3", "95": "left_thumb4",
            "96": "left_forefinger1", "97": "left_forefinger2", "98": "left_forefinger3", "99": "left_forefinger4",
            "100": "left_middle_finger1", "101": "left_middle_finger2", "102": "left_middle_finger3", "103": "left_middle_finger4",
            "104": "left_ring_finger1", "105": "left_ring_finger2", "106": "left_ring_finger3", "107": "left_ring_finger4",
            "108": "left_pinky_finger1", "109": "left_pinky_finger2", "110": "left_pinky_finger3", "111": "left_pinky_finger4",
            "112": "right_hand_root", "113": "right_thumb1", "114": "right_thumb2", "115": "right_thumb3", "116": "right_thumb4",
            "117": "right_forefinger1", "118": "right_forefinger2", "119": "right_forefinger3", "120": "right_forefinger4",
            "121": "right_middle_finger1", "122": "right_middle_finger2", "123": "right_middle_finger3", "124": "right_middle_finger4",
            "125": "right_ring_finger1", "126": "right_ring_finger2", "127": "right_ring_finger3", "128": "right_ring_finger4",
            "129": "right_pinky_finger1", "130": "right_pinky_finger2", "131": "right_pinky_finger3", "132": "right_pinky_finger4"
        }.items()
    },
    
    # skeleton till first bone of each finger
    "whole_body_skeleton_links": [
        [15, 13], [13, 11], [16, 14], [14, 12], [11, 12],
        [5, 11], [6, 12], [5, 6], [5, 7], [6, 8],
        [7, 9], [8, 10], [1, 2], [0, 1], [0, 2],
        [1, 3], [2, 4], [3, 5], [4, 6],
        [15, 17], [15, 18], [15, 19],
        [16, 20], [16, 21], [16, 22],
        
        # Left hand (up to thumb1, finger1)
        [91, 95],     # thumb4
        [91, 103],    # middle_finger4
        [91, 111],    # pinky_finger4

        # Right hand (up to thumb1, finger1)
        [112, 116],   # thumb4
        [112, 124],   # middle_finger4
        [112, 132]    # pinky_finger4
    ],

    "flip_indices": [
        0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15,
        20, 21, 22, 17, 18, 19, 39, 38, 37, 36, 35, 34, 33, 32, 31,
        30, 29, 28, 27, 26, 25, 24, 23, 49, 48, 47, 46, 45, 44, 43,
        42, 41, 40, 50, 51, 52, 53, 58, 57, 56, 55, 54, 68, 67, 66,
        65, 70, 69, 62, 61, 60, 59, 64, 63, 77, 76, 75, 74, 73, 72,
        71, 82, 81, 80, 79, 78, 87, 86, 85, 84, 83, 90, 89, 88, 112,
        113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124,
        125, 126, 127, 128, 129, 130, 131, 132, 91, 92, 93, 94, 95,
        96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107,
        108, 109, 110, 111
    ]
}

def _remap_skeleton_links(links, original_indices):
    """Remap skeleton links from original COCO IDs to 0-indexed filtered positions"""
    index_map = {orig_id: pos for pos, orig_id in enumerate(original_indices)}
    remapped = []
    for a, b in links:
        if a in index_map and b in index_map:
            remapped.append([index_map[a], index_map[b]])
    return remapped

coco_wholebody["upper_body_with_hips_original_indices"] = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 91, 95, 103, 111, 112, 116, 124, 132
]
coco_wholebody["upper_body_with_hips_skeleton_links_remapped"] = _remap_skeleton_links(
    coco_wholebody["whole_body_skeleton_links"], 
    coco_wholebody["upper_body_with_hips_original_indices"]
)

def get_filtered_keypoint_index(joint_name, kpts_config, coco_wholebody_dict):
    """
    Get the index of a keypoint in the filtered array.
    
    Args:
        joint_name: Name of the keypoint (e.g., 'left_shoulder', 'right_elbow')
        kpts_config: Config name (e.g., 'upper_body_with_hips')
        coco_wholebody_dict: The coco_wholebody dictionary from fit3d_variables
    
    Returns:
        Index in the filtered keypoint array, or None if not found
    """
    # Get original COCO ID
    if joint_name not in coco_wholebody_dict["keypoint_name2id"]:
        return None
    
    original_id = coco_wholebody_dict["keypoint_name2id"][joint_name]
    
    # Get filtered indices
    original_indices = coco_wholebody_dict[f"{kpts_config}_original_indices"]
    
    # Find position in filtered array
    try:
        return original_indices.index(original_id)
    except ValueError:
        return None  # joint not in this config

def make_mlp(dim_list, activations, dropout=0):

    if len(dim_list) == 0 and len(activations) == 0:
        return nn.Identity()

    assert len(dim_list) == len(activations)+1
    
    layers = []
    for dim_in, dim_out, activation in zip(dim_list[:-1], dim_list[1:], activations):
                
        # append layer
        layers.append(nn.Linear(dim_in, dim_out))
        
        # # # # # # # # # # # # 
        # append activations  #
        # # # # # # # # # # # #
            
        activation_list = re.split('-', activation)
        for activation in activation_list:
            
            # first because of "in"
            if 'leakyrelu' in activation:
                layers.append(nn.LeakyReLU(negative_slope=float(re.split('=', activation)[1]), inplace=True))
                
            elif activation == 'relu':
                layers.append(nn.ReLU())
                
            elif activation == "sigmoid":
                layers.append(nn.Sigmoid())
                
            elif activation == "none":
                pass
                                
            elif activation == "batchnorm":
                layers.append(nn.BatchNorm1d(dim_out))    
                        
            else:
                print("unknown activation")
                sys.exit()
            
            if dropout > 0:
                print("dropout")
                layers.append(nn.Dropout(p=dropout))
                   
    return nn.Sequential(*layers)

def normalize_kpts_bbox_inplace(kpts_xy_f32, bbox_xyxy_f32):
    x1, y1, x2, y2 = bbox_xyxy_f32
    w = max(x2 - x1, 1.0)
    h = max(y2 - y1, 1.0)
    kpts_xy_f32[:, 0] = (kpts_xy_f32[:, 0] - x1) / w
    kpts_xy_f32[:, 1] = (kpts_xy_f32[:, 1] - y1) / h
    return kpts_xy_f32

class model(nn.Module):
    def __init__(self):
        super(model, self).__init__()
        
        # data type
        self.device = "cpu"
        
        self.left_arm_idxs = [get_filtered_keypoint_index(name, "upper_body_with_hips", coco_wholebody) 
                                for name in ["left_shoulder", "left_elbow", "left_wrist", 
                                            "left_hand_root", "left_thumb4", "left_middle_finger4", "left_pinky_finger4"]]

        self.right_arm_idxs = [get_filtered_keypoint_index(name, "upper_body_with_hips", coco_wholebody)
                                for name in ["right_shoulder", "right_elbow", "right_wrist",
                                            "right_hand_root", "right_thumb4", "right_middle_finger4", "right_pinky_finger4"]]
        
        self.spine_idxs = [get_filtered_keypoint_index(name, "upper_body_with_hips", coco_wholebody)
                            for name in ["left_hip", "right_hip", "left_shoulder", "right_shoulder", 
                                        "nose", "left_eye", "right_eye", "left_ear", "right_ear"]]
            
        # Input dims from mmpose keypoints
        left_arm_input  = len([x for x in self.left_arm_idxs if x is not None]) * 2   # 7 keypoints × 2
        right_arm_input = len([x for x in self.right_arm_idxs if x is not None]) * 2  # 7 keypoints × 2
        spine_input     = len([x for x in self.spine_idxs if x is not None]) * 2      # 9 keypoints × 2
        
        self.left_arm_mlp   = make_mlp([left_arm_input] + [128] + [10], ["relu", "none"])
        self.right_arm_mlp  = make_mlp([right_arm_input] + [128] + [10], ["relu", "none"])
        self.spine_mlp      = make_mlp([spine_input] + [256] + [30], ["relu", "none"])
        
    def forward(self, data):
                                 
        # Extract body parts using the computed indices
        left_arm    = data[:, self.left_arm_idxs, :].reshape(1, -1)   # [1, num_left_arm_kpts*2]
        right_arm   = data[:, self.right_arm_idxs, :].reshape(1, -1)  # [1, num_right_arm_kpts*2]
        spine       = data[:, self.spine_idxs, :].reshape(1, -1)      # [1, num_spine_kpts*2]

        left_arm_out    = self.left_arm_mlp(left_arm)   # [1, output_dim]
        right_arm_out   = self.right_arm_mlp(right_arm) # [1, output_dim]
        spine_out       = self.spine_mlp(spine)         # [1, output_dim]

        # Concatenate and pad to 130 (zeros for legs/fingers)
        pred_body_params = torch.zeros(1, 130).to(device="cpu")
        pred_body_params[:, 24:34]     = right_arm_out     # right arm
        pred_body_params[:, 34:44]     = left_arm_out      # left arm
        pred_body_params[:, 0:24]      = spine_out[:, :24] # spine/neck/head
        pred_body_params[:, 124:130]   = spine_out[:, 24:] # shared params

        return pred_body_params

class MHRSubscriber(Node):
    def __init__(self):
        super().__init__('mhr_subscriber')

        ##### Network
        self.net = model()
        self.net.type(torch.FloatTensor)
        weights_path = "/home/haziq/ros2_ws/src/booster_teleop/all_epoch_0960_best_0960_state_dict.pt"
        ckpt = torch.load(weights_path, map_location="cpu", weights_only=False)
        if isinstance(ckpt, dict) and "model_state" in ckpt:
            state = ckpt["model_state"]
        elif isinstance(ckpt, dict) and "state_dict" in ckpt:
            state = ckpt["state_dict"]
        else:
            state = ckpt
        missing, unexpected = self.net.load_state_dict(state, strict=True)
        print(f"[WEIGHTS] loaded {weights_path}")
        print(f"[WEIGHTS] missing={len(missing)} unexpected={len(unexpected)}")
        
        ##### ROS2 Subscriber
        self.subscription = self.create_subscription(
            String,
            '/human_keypoints',
            self.keypoints_callback,
            10
        )

        ##### ROS2 Publisher for joint angles
        self.joint_pub = self.create_publisher(JointState, '/ik/joint_states', 10)
        
        ##### Load TorchScript MHR model
        self.get_logger().info('Loading TorchScript MHR model...')
        model_path = "/home/haziq/MHR/assets/mhr_model.pt" #os.path.join(os.path.dirname(__file__), 'assets', 'mhr_model.pt')
        self.mhr_model = torch.jit.load(model_path, map_location='cpu')
        self.get_logger().info('TorchScript MHR model loaded successfully')
        
        self.frame_count = 0

    def keypoints_callback(self, msg):
        """Process received keypoints and run MHR forward pass."""
        #print(f"[SUBSCRIBER] Received /human_keypoints: {msg.data}")
        try:
            
            # Parse JSON data
            data        = json.loads(msg.data)
            frame_num   = data['frame']
            people      = data['people']
            
            if len(people) == 0:
                return  # No people detected
            
            ##### process 2D keypoints for input to self.net
            person  = people[0]
            kpts    = np.array(person['keypoints'])     # Shape: [133, 3]
            kpts_xy, scores = kpts[:, :2], kpts[:, 2]   # Shape: [133, 2], [133]
            bbox    = person["bbox"]                    # [x1, y1, x2, y2]
            normalize_kpts_bbox_inplace(kpts_xy, bbox)
            
            kpts_xy = kpts_xy[coco_wholebody["upper_body_with_hips_original_indices"]]  # Shape: [21, 2]
            scores = scores[coco_wholebody["upper_body_with_hips_original_indices"]]    # Shape: [21]
            kpts_xy[scores < 0.5] = 0.0                                                 # Zero out low-confidence keypoints
            kpts_xy = torch.from_numpy(kpts_xy.astype(np.float32)).unsqueeze(0)         # Shape: [1, 21, 2]
            
            ##### mlp forward pass
            with torch.inference_mode():
                body_pose_params = self.net(kpts_xy)[0] # Shape: [130]
            
            batch_size = 1
            identity_coeffs     = torch.zeros(batch_size, 45)   # Dummy identity
            model_parameters    = torch.cat([
                        torch.zeros((6), dtype=body_pose_params.dtype), 
                        body_pose_params,
                        torch.zeros((68), dtype=body_pose_params.dtype)], axis=0)
            model_parameters    = model_parameters.unsqueeze(0) # Shape: [1, 204]
            face_expr_coeffs = torch.zeros(batch_size, 72)      # Dummy expression
            
            ##### mhr forward pass
            with torch.no_grad():
                vertices, skeleton_state = self.mhr_model(
                    identity_coeffs,
                    model_parameters,
                    face_expr_coeffs
                )

            # All 127 body joint names ===
            # Anchors:       0 body_world, 1 root
            # L leg:         2-8 (upleg, lowleg, foot, talocrural, subtalar, transversetarsal, ball) + 9-17 twist procs
            # R leg:        18-24 (upleg, lowleg, foot, talocrural, subtalar, transversetarsal, ball) + 25-33 twist procs
            # Spine:        34-37 (c_spine0, c_spine1, c_spine2, c_spine3)
            # R arm:        38-42 (clavicle, uparm, lowarm, wrist_twist, wrist) + 65-73 twist procs
            # R hand:       43-64 (pinky0-3, ring1-3, middle1-3, index1-3, thumb0-3 + nulls)
            # L arm:        74-78 (clavicle, uparm, lowarm, wrist_twist, wrist) + 101-109 twist procs
            # L hand:       79-100 (pinky0-3, ring1-3, middle1-3, index1-3, thumb0-3 + nulls)
            # Head/face:   110-126 (neck, neck_twist, head, jaw, teeth, tongue0-4, eyes, nulls)
            _, joint_quats, _   = torch.split(skeleton_state, [3, 4, 1], dim=2)
            joint_rots          = roma.unitquat_to_rotmat(joint_quats)[0]   # Shape: [127, 3, 3]
            
            # AAHead_yaw
            # Head_pitch
            # Left_Ankle_Pitch
            # Left_Ankle_Roll
            # (X) Left_Elbow_Pitch
            # (X) Left_Elbow_Yaw
            # (X) Left_Hand_Roll
            # (X) Left_Hip_Pitch
            # Left_Hip_Roll
            # Left_Hip_Yaw
            # Left_Knee_Pitch
            # (X) Left_Shoulder_Pitch
            # (X) Left_Shoulder_Roll
            # (X) Left_Wrist_Pitch
            # (X) Left_Wrist_Yaw
            # Right_Ankle_Pitch
            # Right_Ankle_Roll
            # (X) Right_Elbow_Pitch
            # (X) Right_Elbow_Yaw
            # (X) Right_Hand_Roll
            # Right_Hip_Pitch
            # Right_Hip_Roll
            # Right_Hip_Yaw
            # Right_Knee_Pitch
            # (X) Right_Shoulder_Pitch
            # (X) Right_Shoulder_Roll
            # (X) Right_Wrist_Pitch
            # (X) Right_Wrist_Yaw

                
            # Log results
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Frame {frame_num}: Processed {len(people)} people, '
                    f'MHR output vertices shape: {vertices.shape}, '
                    f'skeleton_state shape: {skeleton_state.shape}'
                )
            
            # Example: Extract left elbow pitch and yaw from joint_rots[76]
            left_elbow_rot = joint_rots[76]  # [3,3] matrix for left lowarm
            # Convert to Euler angles (order: yaw, pitch, roll)
            euler_angles = Rotation.from_matrix(left_elbow_rot).as_euler('zyx', degrees=False)
            left_elbow_yaw = euler_angles[0]   # rotation about z-axis
            left_elbow_pitch = euler_angles[1] # rotation about y-axis

            # Publish to /ik/joint_states
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['left_elbow_yaw', 'left_elbow_pitch']  # Replace with your robot's actual joint names if different
            joint_msg.position = [float(left_elbow_yaw), float(left_elbow_pitch)]
            self.joint_pub.publish(joint_msg)
            print(f"Left elbow yaw: {left_elbow_yaw:.3f} rad, pitch: {left_elbow_pitch:.3f} rad (published)")
            
        except Exception as e:
            self.get_logger().info(f'Error processing keypoints: {e}')

def main(args=None):

    rclpy.init(args=args)
    node = MHRSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()