#!/usr/bin/env python3
"""
MMPose Publisher Node (Python 3.8)
Runs pose tracker on webcam and publishes keypoints to ROS topic.

# Author: Haziq
# Date Created: 30 Jan 2026
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import json
from mmdeploy_runtime import PoseTracker


class MMPosePublisher(Node):
    def __init__(self):
        super().__init__('mmpose_publisher')
        
        # ROS2 Publisher
        self.publisher = self.create_publisher(String, '/human_keypoints', 10)
        
        # MMDeploy PoseTracker setup
        det_model = 'src/booster_teleop/rtmpose-ort/rtmdet-nano'
        pose_model = 'src/booster_teleop/rtmpose-ort/rtmw-dw-l-m'
        
        self.get_logger().info('Initializing PoseTracker...')
        self.tracker = PoseTracker(
            det_model=det_model,
            pose_model=pose_model,
            device_name='cpu',
            device_id=0
        )
        
        # Initialize tracker state for person tracking across frames
        self.tracker_state = self.tracker.create_state(det_interval=10, det_min_bbox_size=100)
        
        # Webcam setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().info('Failed to open webcam!')
            raise RuntimeError('Webcam not available')
        
        self.get_logger().info('Webcam opened successfully')
        
        # Timer for processing frames (30 Hz)
        self.timer = self.create_timer(0.033, self.process_frame)
        
        self.frame_count = 0
        
        # Visualization settings (COCO skeleton)
        self.skeleton = [(15, 13), (13, 11), (16, 14), (14, 12), (11, 12), (5, 11),
                         (6, 12), (5, 6), (5, 7), (6, 8), (7, 9), (8, 10), (1, 2),
                         (0, 1), (0, 2), (1, 3), (2, 4), (3, 5), (4, 6)]
        self.palette = [(255, 128, 0), (255, 153, 51), (255, 178, 102), (230, 230, 0),
                        (255, 153, 255), (153, 204, 255), (255, 102, 255),
                        (255, 51, 255), (102, 178, 255), (51, 153, 255),
                        (255, 153, 153), (255, 102, 102), (255, 51, 51),
                        (153, 255, 153), (102, 255, 102), (51, 255, 51), (0, 255, 0),
                        (0, 0, 255), (255, 0, 0), (255, 255, 255)]
        
        self.link_color = [0, 0, 0, 0, 7, 7, 7, 9, 9, 9, 9, 9, 16, 16, 16, 16, 16, 16, 16]
        self.point_color = [16, 16, 16, 16, 16, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0]
        self.kpt_thr = 0.5  # Keypoint confidence threshold

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Failed to read frame')
            return
        
        # Run pose detection with state tracking
        # Returns tuple: (keypoints, bboxes, target_ids) - state is updated in-place
        keypoints, bboxes, target_ids = self.tracker(self.tracker_state, frame, detect=-1)
        
        # Visualize on frame
        vis_frame = self.visualize(frame.copy(), keypoints, bboxes)
        cv2.imshow('MMPose Publisher', vis_frame)
        cv2.waitKey(1)
        
        # Extract keypoints from all detected people
        keypoints_data = []
        for target_id, bbox, kpts in zip(target_ids, bboxes, keypoints):
            person_data = {
                'target_id': int(target_id),
                'bbox': bbox.tolist(),
                'keypoints': kpts.tolist()  # Shape: [num_keypoints, 3] (x, y, confidence)
            }
            keypoints_data.append(person_data)
        
        # Publish as JSON string
        msg = String()
        msg.data = json.dumps({
            'frame': self.frame_count,
            'people': keypoints_data
        })
        self.publisher.publish(msg)
        
        self.frame_count += 1
        
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames, detected {len(keypoints_data)} people')
    
    def visualize(self, frame, keypoints, bboxes):
        """Draw keypoints and skeleton on frame"""
        for kpts, bbox in zip(keypoints, bboxes):
            # Draw bounding box
            x1, y1, x2, y2 = bbox.astype(int)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw skeleton connections
            for (u, v), color_idx in zip(self.skeleton, self.link_color):
                if u < len(kpts) and v < len(kpts):
                    if kpts[u][2] > self.kpt_thr and kpts[v][2] > self.kpt_thr:
                        pt1 = tuple(kpts[u][:2].astype(int))
                        pt2 = tuple(kpts[v][:2].astype(int))
                        cv2.line(frame, pt1, pt2, self.palette[color_idx], 2, cv2.LINE_AA)
            
            # Draw keypoints
            for kpt, color_idx in zip(kpts, self.point_color):
                if kpt[2] > self.kpt_thr:
                    pt = tuple(kpt[:2].astype(int))
                    cv2.circle(frame, pt, 3, self.palette[color_idx], -1, cv2.LINE_AA)
        
        return frame

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MMPosePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()