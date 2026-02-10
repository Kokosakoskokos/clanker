#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time
import numpy as np
import yaml
import os


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        self.get_logger().info('Initializing LiDAR Node...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/navigation.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        
        self.serial_port = None
        self.init_lidar()
        
        self.timer = self.create_timer(
            0.1,
            self.read_and_publish_scan
        )
        
        self.get_logger().info('LiDAR Node initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'frame_id': 'lidar_link',
            'min_range': 0.12,
            'max_range': 12.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.01745
        }
    
    def init_lidar(self):
        try:
            self.serial_port = serial.Serial(
                self.config['port'],
                self.config['baudrate'],
                timeout=1
            )
            self.get_logger().info(f'LiDAR connected on {self.config["port"]}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to LiDAR: {e}')
            self.get_logger().warn('Running in simulation mode')
    
    def read_and_publish_scan(self):
        if self.serial_port is None:
            self.publish_simulation_scan()
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.read(self.serial_port.in_waiting)
                self.process_scan_data(data)
        except Exception as e:
            self.get_logger().error(f'Error reading from LiDAR: {e}')
    
    def process_scan_data(self, data):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.config['frame_id']
        
        scan.angle_min = self.config['angle_min']
        scan.angle_max = self.config['angle_max']
        scan.angle_increment = self.config['angle_increment']
        
        num_samples = int(
            (scan.angle_max - scan.angle_min) / scan.angle_increment
        )
        
        scan.ranges = [self.config['max_range']] * num_samples
        scan.intensities = [0.0] * num_samples
        
        scan.range_min = self.config['min_range']
        scan.range_max = self.config['max_range']
        
        self.scan_pub.publish(scan)
    
    def publish_simulation_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.config['frame_id']
        
        scan.angle_min = self.config['angle_min']
        scan.angle_max = self.config['angle_max']
        scan.angle_increment = self.config['angle_increment']
        
        num_samples = int(
            (scan.angle_max - scan.angle_min) / scan.angle_increment
        )
        
        angles = np.linspace(
            scan.angle_min,
            scan.angle_max,
            num_samples
        )
        
        ranges = []
        for angle in angles:
            dist = self.config['max_range'] - 0.5 * (np.sin(angle) + 1)
            dist = max(self.config['min_range'], dist)
            ranges.append(dist)
        
        scan.ranges = ranges
        scan.intensities = [0.0] * num_samples
        
        scan.range_min = self.config['min_range']
        scan.range_max = self.config['max_range']
        
        self.scan_pub.publish(scan)
    
    def destroy_node(self):
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
