#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.get_logger().info('Initializing Camera Node...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/camera.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.bridge = CvBridge()
        
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/camera_info',
            10
        )
        
        self.cap = cv2.VideoCapture(self.config['camera_index'])
        
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config['width'])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config['height'])
        self.cap.set(cv2.CAP_PROP_FPS, self.config['fps'])
        
        self.camera_info = self.create_camera_info()
        
        self.timer = self.create_timer(
            1.0 / self.config['fps'],
            self.capture_and_publish
        )
        
        self.get_logger().info('Camera Node initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'camera_index': 0,
            'width': 640,
            'height': 480,
            'fps': 30
        }
    
    def create_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = 'camera_link'
        info.width = self.config['width']
        info.height = self.config['height']
        
        fx = fy = info.width / (2 * math.tan(math.pi / 6))
        cx, cy = info.width / 2, info.height / 2
        
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        return info
    
    def capture_and_publish(self):
        try:
            ret, frame = self.cap.read()
            
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_link'
                self.image_pub.publish(msg)
                
                self.camera_info.header.stamp = msg.header.stamp
                self.camera_info_pub.publish(self.camera_info)
            else:
                self.get_logger().warn('Failed to capture frame')
                
        except Exception as e:
            self.get_logger().error(f'Error capturing frame: {e}')
    
    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    import math
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
