#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import yaml
import os


class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        
        self.get_logger().info('Initializing Navigator...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/navigation.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        self.distance_subs = {}
        for direction in ['front', 'left', 'right', 'rear']:
            topic = f'/{direction}_distance'
            self.distance_subs[direction] = self.create_subscription(
                Range,
                topic,
                lambda msg, d=direction: self.ultrasonic_callback(msg, d),
                10
            )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.nav_command_sub = self.create_subscription(
            String,
            '/navigation_command',
            self.nav_command_callback,
            10
        )
        
        self.obstacle_detected_pub = self.create_publisher(
            String,
            '/obstacle_detected',
            10
        )
        
        self.mode = 'autonomous'
        self.target_velocity = Twist()
        self.obstacle_distances = {}
        self.lidar_data = None
        
        self.timer = self.create_timer(
            0.1,
            self.navigate
        )
        
        self.get_logger().info('Navigator initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'safety_distance': 0.5,
            'slow_distance': 1.0,
            'max_linear_speed': 0.3,
            'max_angular_speed': 0.5,
            'obstacle_threshold': 0.3
        }
    
    def lidar_callback(self, msg):
        self.lidar_data = msg
    
    def ultrasonic_callback(self, msg, direction):
        self.obstacle_distances[direction] = msg.range
    
    def nav_command_callback(self, msg):
        parts = msg.data.split()
        
        if len(parts) >= 1:
            command = parts[0]
            
            if command == 'stop':
                self.mode = 'stopped'
            elif command == 'autonomous':
                self.mode = 'autonomous'
            elif command == 'forward':
                self.mode = 'manual'
                speed = float(parts[1]) if len(parts) > 1 else 0.2
                self.target_velocity.linear.x = speed
                self.target_velocity.angular.z = 0.0
            elif command == 'backward':
                self.mode = 'manual'
                speed = float(parts[1]) if len(parts) > 1 else 0.2
                self.target_velocity.linear.x = -speed
                self.target_velocity.angular.z = 0.0
            elif command == 'left':
                self.mode = 'manual'
                speed = float(parts[1]) if len(parts) > 1 else 0.3
                self.target_velocity.linear.x = 0.0
                self.target_velocity.angular.z = speed
            elif command == 'right':
                self.mode = 'manual'
                speed = float(parts[1]) if len(parts) > 1 else 0.3
                self.target_velocity.linear.x = 0.0
                self.target_velocity.angular.z = -speed
    
    def navigate(self):
        if self.mode == 'stopped':
            self.stop_robot()
            return
        
        if self.mode == 'manual':
            safe_velocity = self.check_obstacles(self.target_velocity)
            self.cmd_vel_pub.publish(safe_velocity)
            return
        
        if self.mode == 'autonomous':
            self.autonomous_navigation()
    
    def check_obstacles(self, velocity):
        safe_velocity = Twist()
        safe_velocity.linear.x = velocity.linear.x
        safe_velocity.angular.z = velocity.angular.z
        
        safety_distance = self.config['safety_distance']
        slow_distance = self.config['slow_distance']
        
        front_distance = self.obstacle_distances.get('front', 10.0)
        
        if front_distance < safety_distance:
            safe_velocity.linear.x = 0.0
            safe_velocity.angular.z = 0.5
            self.obstacle_detected_pub.publish(String(data='front'))
        elif front_distance < slow_distance:
            safe_velocity.linear.x *= 0.3
        
        if self.lidar_data:
            min_distance = min(self.lidar_data.ranges)
            if min_distance < safety_distance:
                self.avoid_obstacle_lidar(min_distance)
        
        return safe_velocity
    
    def avoid_obstacle_lidar(self, min_distance):
        if not self.lidar_data:
            return
        
        num_samples = len(self.lidar_data.ranges)
        center_idx = num_samples // 2
        
        left_distances = self.lidar_data.ranges[:center_idx]
        right_distances = self.lidar_data.ranges[center_idx:]
        
        left_avg = np.mean(left_distances)
        right_avg = np.mean(right_distances)
        
        avoid_velocity = Twist()
        avoid_velocity.linear.x = 0.1
        
        if left_avg > right_avg:
            avoid_velocity.angular.z = 0.3
        else:
            avoid_velocity.angular.z = -0.3
        
        self.cmd_vel_pub.publish(avoid_velocity)
    
    def autonomous_navigation(self):
        exploration_velocity = Twist()
        exploration_velocity.linear.x = self.config['max_linear_speed'] * 0.5
        
        safe_velocity = self.check_obstacles(exploration_velocity)
        self.cmd_vel_pub.publish(safe_velocity)
    
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
