#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import dlib
import pickle
import os
import yaml
from pathlib import Path


class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        
        self.get_logger().info('Initializing Face Recognition Node...')
        
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
        
        self.face_detection_pub = self.create_publisher(
            Detection2DArray,
            '/face_detections',
            10
        )
        
        self.recognition_pub = self.create_publisher(
            String,
            '/recognized_faces',
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/face_command',
            self.face_command_callback,
            10
        )
        
        self.init_face_detector()
        self.init_face_recognizer()
        self.load_known_faces()
        
        self.tracking_enabled = self.config.get('person_tracking', {}).get('enabled', True)
        self.tracked_person = None
        self.tracking_lost_counter = 0
        
        self.get_logger().info('Face Recognition Node initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'face_detection': {
                'enabled': True,
                'model': 'opencv_haar',
                'haar_cascade_path': 'models/haarcascade_frontalface_default.xml',
                'confidence_threshold': 0.8,
                'min_face_size': 50,
                'max_faces': 5
            },
            'face_recognition': {
                'enabled': True,
                'encoding_model': 'dlib',
                'tolerance': 0.6
            },
            'person_tracking': {
                'enabled': True,
                'tracking_method': 'centroid',
                'max_distance': 100
            }
        }
    
    def init_face_detector(self):
        try:
            face_config = self.config.get('face_detection', {})
            
            if face_config.get('model', 'opencv_haar') == 'opencv_haar':
                cascade_path = face_config.get('haar_cascade_path', 
                    'models/haarcascade_frontalface_default.xml')
                self.face_cascade = cv2.CascadeClassifier(cascade_path)
                
                if self.face_cascade.empty():
                    self.get_logger().warn('Haar cascade not found, using default')
                    self.face_cascade = cv2.CascadeClassifier(
                        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                    )
                
                self.get_logger().info('Haar cascade face detector loaded')
            else:
                self.face_detector = dlib.get_frontal_face_detector()
                self.get_logger().info('Dlib HOG face detector loaded')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize face detector: {e}')
            self.face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            )
    
    def init_face_recognizer(self):
        try:
            face_config = self.config.get('face_recognition', {})
            
            if face_config.get('encoding_model', 'dlib') == 'dlib':
                predictor_path = face_config.get('face_landmark_model',
                    'models/shape_predictor_68_face_landmarks.dat')
                recognizer_path = face_config.get('face_recognition_model',
                    'models/dlib_face_recognition_resnet_model_v1.dat')
                
                self.face_predictor = dlib.shape_predictor(predictor_path)
                self.face_recognizer = dlib.face_recognition_model_v1(recognizer_path)
                
                self.get_logger().info('Dlib face recognizer loaded')
            else:
                self.face_recognizer = cv2.face.LBPHFaceRecognizer_create()
                self.get_logger().info('OpenCV LBPH face recognizer loaded')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize face recognizer: {e}')
            self.get_logger().warn('Recognition may not work properly')
    
    def load_known_faces(self):
        self.known_faces = {}
        self.known_encodings = {}
        
        known_faces_dir = os.path.join(
            os.path.dirname(__file__),
            '../../../data/known_faces'
        )
        
        if not os.path.exists(known_faces_dir):
            os.makedirs(known_faces_dir, exist_ok=True)
            self.get_logger().info(f'Created directory: {known_faces_dir}')
            return
        
        try:
            encoding_file = os.path.join(known_faces_dir, 'encodings.pkl')
            if os.path.exists(encoding_file):
                with open(encoding_file, 'rb') as f:
                    data = pickle.load(f)
                    self.known_encodings = data.get('encodings', {})
                    self.known_faces = data.get('names', {})
                
                self.get_logger().info(f'Loaded {len(self.known_faces)} known faces')
            else:
                self.get_logger().info('No known faces database found')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load known faces: {e}')
    
    def save_known_faces(self):
        known_faces_dir = os.path.join(
            os.path.dirname(__file__),
            '../../../data/known_faces'
        )
        os.makedirs(known_faces_dir, exist_ok=True)
        
        try:
            encoding_file = os.path.join(known_faces_dir, 'encodings.pkl')
            data = {
                'encodings': self.known_encodings,
                'names': self.known_faces
            }
            with open(encoding_file, 'wb') as f:
                pickle.dump(data, f)
            
            self.get_logger().info('Known faces saved')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save known faces: {e}')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            faces = self.detect_faces(cv_image)
            
            detections = Detection2DArray()
            detections.header = msg.header
            
            recognized_names = []
            
            for i, face in enumerate(faces):
                detection = Detection2D()
                detection.header = msg.header
                
                if isinstance(face, tuple):
                    x, y, w, h = face
                    detection.bbox.center.x = x + w / 2
                    detection.bbox.center.y = y + h / 2
                    detection.bbox.size_x = w
                    detection.bbox.size_y = h
                    
                    name, confidence = self.recognize_face(cv_image, face)
                    
                    if name:
                        recognized_names.append(name)
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = str(i)
                        hypothesis.hypothesis.score = confidence
                        hypothesis.hypothesis.class_name = name
                        detection.results.append(hypothesis)
                    
                    detections.detections.append(detection)
            
            self.face_detection_pub.publish(detections)
            
            if recognized_names:
                names_msg = String()
                names_msg.data = ', '.join(recognized_names)
                self.recognition_pub.publish(names_msg)
                
                if self.tracking_enabled:
                    self.update_tracking(recognized_names, faces, cv_image.shape)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def detect_faces(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = []
        
        face_config = self.config.get('face_detection', {})
        
        if hasattr(self, 'face_cascade') and self.face_cascade is not None:
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(face_config.get('min_face_size', 50), 
                        face_config.get('min_face_size', 50))
            )
        elif hasattr(self, 'face_detector') and self.face_detector is not None:
            dlib_faces = self.face_detector(gray)
            faces = [(f.left(), f.top(), f.width(), f.height()) for f in dlib_faces]
        
        return faces
    
    def recognize_face(self, cv_image, face_rect):
        if not hasattr(self, 'face_recognizer') or self.face_recognizer is None:
            return None, 0.0
        
        x, y, w, h = face_rect
        face_roi = cv_image[y:y+h, x:x+w]
        
        try:
            if isinstance(self.face_recognizer, dlib.face_recognition_model_v1):
                dlib_rect = dlib.rectangle(int(x), int(y), int(x+w), int(y+h))
                shape = self.face_predictor(cv_image, dlib_rect)
                encoding = np.array(self.face_recognizer.compute_face_descriptor(
                    cv_image, shape))
                
                name, confidence = self.compare_encodings(encoding)
                return name, confidence
            else:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                face_gray = gray[y:y+h, x:x+w]
                label, confidence = self.face_recognizer.predict(face_gray)
                
                if confidence < 100:
                    return self.known_faces.get(label, 'Unknown'), confidence
                return None, 0.0
                
        except Exception as e:
            self.get_logger().error(f'Error recognizing face: {e}')
            return None, 0.0
    
    def compare_encodings(self, encoding):
        if not self.known_encodings:
            return 'Unknown', 0.0
        
        tolerance = self.config.get('face_recognition', {}).get('tolerance', 0.6)
        
        best_match = 'Unknown'
        best_distance = float('inf')
        
        for name, known_encoding in self.known_encodings.items():
            distance = np.linalg.norm(encoding - np.array(known_encoding))
            
            if distance < best_distance and distance < tolerance:
                best_distance = distance
                best_match = name
        
        confidence = max(0.0, 1.0 - best_distance)
        return best_match, confidence
    
    def face_command_callback(self, msg):
        parts = msg.data.split()
        
        if len(parts) >= 2:
            command = parts[0]
            
            if command == 'register':
                name = parts[1]
                self.register_face_command(name)
            elif command == 'forget':
                name = parts[1]
                self.forget_face(name)
            elif command == 'list':
                self.list_known_faces()
            elif command == 'track':
                if len(parts) >= 3:
                    name = parts[1]
                    self.tracked_person = name
    
    def register_face_command(self, name):
        self.get_logger().info(f'Waiting to register face for: {name}')
        self.pending_registration_name = name
    
    def forget_face(self, name):
        if name in self.known_faces:
            del self.known_faces[name]
        if name in self.known_encodings:
            del self.known_encodings[name]
        
        self.save_known_faces()
        self.get_logger().info(f'Forgot face: {name}')
    
    def list_known_faces(self):
        if self.known_faces:
            names = ', '.join(self.known_faces.keys())
            self.get_logger().info(f'Known faces: {names}')
        else:
            self.get_logger().info('No known faces')
    
    def update_tracking(self, recognized_names, faces, image_shape):
        if self.tracked_person:
            if self.tracked_person in recognized_names:
                self.tracking_lost_counter = 0
            else:
                self.tracking_lost_counter += 1
                
                tracking_config = self.config.get('person_tracking', {})
                lost_threshold = tracking_config.get('lost_threshold', 30)
                
                if self.tracking_lost_counter > lost_threshold:
                    self.get_logger().info(f'Lost track of: {self.tracked_person}')
                    self.tracked_person = None
    
    def destroy_node(self):
        self.save_known_faces()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
