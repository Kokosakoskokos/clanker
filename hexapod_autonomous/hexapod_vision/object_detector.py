#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import yaml
import os
from pathlib import Path


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        self.get_logger().info('Initializing Object Detector...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/camera.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image,
            '/detection_debug',
            10
        )
        
        self.model = None
        self.classes = []
        self.load_model()
        
        self.get_logger().info('Object Detector initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'model_path': 'models/yolov5s.pt',
            'conf_threshold': 0.5,
            'iou_threshold': 0.45,
            'device': 'cpu'
        }
    
    def load_model(self):
        try:
            self.get_logger().info('Loading YOLOv5 model...')
            
            model_path = self.config.get('model_path', 'yolov5s')
            self.model = torch.hub.load(
                'ultralytics/yolov5',
                'custom',
                path=model_path if os.path.exists(model_path) else 'yolov5s',
                device=self.config.get('device', 'cpu')
            )
            
            self.model.conf = self.config.get('conf_threshold', 0.5)
            self.model.iou = self.config.get('iou_threshold', 0.45)
            
            self.classes = self.model.names
            self.get_logger().info(f'Model loaded with {len(self.classes)} classes')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.get_logger().warn('Running without object detection')
    
    def image_callback(self, msg):
        if self.model is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            results = self.model(cv_image)
            
            detections = Detection2DArray()
            detections.header = msg.header
            
            debug_image = cv_image.copy()
            
            for result in results.xyxy[0]:
                x1, y1, x2, y2, conf, cls_idx = result
                
                detection = Detection2D()
                detection.header = msg.header
                
                detection.bbox.center.x = float((x1 + x2) / 2)
                detection.bbox.center.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(int(cls_idx))
                hypothesis.hypothesis.score = float(conf)
                
                if int(cls_idx) < len(self.classes):
                    class_name = self.classes[int(cls_idx)]
                    detection.results.append(hypothesis)
                
                detections.detections.append(detection)
                
                cv2.rectangle(
                    debug_image,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    (0, 255, 0),
                    2
                )
                
                if int(cls_idx) < len(self.classes):
                    label = f'{self.classes[int(cls_idx)]} {conf:.2f}'
                    cv2.putText(
                        debug_image,
                        label,
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
            
            self.detection_pub.publish(detections)
            
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
