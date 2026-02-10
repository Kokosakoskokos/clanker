#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')
        
        self.get_logger().info('Initializing Person Follower...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/navigation.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.bridge = CvBridge()
        
        self.face_detection_sub = self.create_subscription(
            Detection2DArray,
            '/face_detections',
            self.face_detection_callback,
            10
        )
        
        self.recognition_sub = self.create_subscription(
            String,
            '/recognized_faces',
            self.recognition_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/person_follow_command',
            self.follow_command_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/follow_status',
            10
        )
        
        person_config = self.config.get('person_following', {})
        
        self.follow_distance = person_config.get('follow_distance', 1.5)
        self.min_distance = person_config.get('min_distance', 0.8)
        self.max_distance = person_config.get('max_distance', 3.0)
        self.lost_timeout = person_config.get('lost_timeout', 5.0)
        self.face_confidence_threshold = person_config.get('face_confidence_threshold', 0.7)
        
        self.mode = 'idle'
        self.target_person = None
        self.last_detection_time = 0
        self.target_center = None
        self.target_size = None
        
        self.linear_speed = 0.15
        self.angular_speed = 0.3
        
        self.timer = self.create_timer(0.1, self.update_following)
        
        self.get_logger().info('Person Follower initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'person_following': {
                'enabled': True,
                'follow_distance': 1.5,
                'min_distance': 0.8,
                'max_distance': 3.0,
                'lost_timeout': 5.0,
                'face_confidence_threshold': 0.7
            }
        }
    
    def face_detection_callback(self, msg):
        if not msg.detections:
            return
        
        detection = msg.detections[0]
        
        if not detection.results:
            return
        
        result = detection.results[0]
        confidence = result.hypothesis.score
        
        if confidence < self.face_confidence_threshold:
            return
        
        if self.target_person and result.hypothesis.class_name == self.target_person:
            self.update_target(detection)
        elif not self.target_person:
            self.update_target(detection)
    
    def recognition_callback(self, msg):
        names = [name.strip() for name in msg.data.split(',')]
        
        if self.target_person and self.target_person in names:
            self.publish_status(f'following_{self.target_person}')
    
    def follow_command_callback(self, msg):
        command = msg.data.lower()
        
        if command == 'start':
            self.mode = 'following'
            self.publish_status('started')
        elif command == 'stop':
            self.mode = 'idle'
            self.target_person = None
            self.stop_robot()
            self.publish_status('stopped')
        elif command.startswith('follow '):
            name = command.replace('follow ', '').strip()
            self.target_person = name
            self.mode = 'following'
            self.publish_status(f'following_{name}')
        elif command == 'auto':
            self.mode = 'auto'
            self.target_person = None
            self.publish_status('auto')
    
    def update_target(self, detection):
        self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
        
        center_x = detection.bbox.center.x
        center_y = detection.bbox.center.y
        width = detection.bbox.size_x
        height = detection.bbox.size_y
        
        self.target_center = (center_x, center_y)
        self.target_size = (width, height)
    
    def update_following(self):
        if self.mode == 'idle':
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        if current_time - self.last_detection_time > self.lost_timeout:
            self.mode = 'searching'
            self.publish_status('lost')
            return
        
        if self.mode == 'searching' and self.target_center:
            self.mode = 'following'
        
        if self.target_center:
            self.follow_target()
        else:
            self.stop_robot()
    
    def follow_target(self):
        if not self.target_center or not self.target_size:
            self.stop_robot()
            return
        
        center_x, center_y = self.target_center
        width, height = self.target_size
        
        image_center_x = 320
        image_center_y = 240
        
        x_offset = center_x - image_center_x
        y_offset = center_y - image_center_y
        
        face_size = max(width, height)
        
        cmd_vel = Twist()
        
        if face_size < 100:
            cmd_vel.linear.x = self.linear_speed * 0.8
        elif face_size > 200:
            cmd_vel.linear.x = -self.linear_speed * 0.5
        else:
            cmd_vel.linear.x = 0.0
        
        if abs(x_offset) > 50:
            if x_offset > 0:
                cmd_vel.angular.z = -self.angular_speed * (abs(x_offset) / 320)
            else:
                cmd_vel.angular.z = self.angular_speed * (abs(x_offset) / 320)
        else:
            cmd_vel.angular.z = 0.0
        
        if abs(y_offset) > 30:
            if y_offset > 0:
                cmd_vel.angular.z += 0.1
            else:
                cmd_vel.angular.z -= 0.1
        
        cmd_vel.linear.x = np.clip(cmd_vel.linear.x, -0.2, 0.2)
        cmd_vel.angular.z = np.clip(cmd_vel.angular.z, -0.5, 0.5)
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        if self.mode == 'following' and self.target_person:
            self.publish_status(f'following_{self.target_person}')
    
    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
    
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
