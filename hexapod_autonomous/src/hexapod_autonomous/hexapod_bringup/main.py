#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import yaml
import os


class HexapodBringup(Node):
    def __init__(self):
        super().__init__('hexapod_bringup')
        
        self.get_logger().info('Initializing Hexapod Bringup...')
        
        self.status_pub = self.create_publisher(
            String,
            '/system_status',
            10
        )
        
        self.command_sub = self.create_subscription(
            String,
            '/system_command',
            self.system_command_callback,
            10
        )
        
        self.timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info('Hexapod Bringup initialized')
        self.get_logger().info('All systems ready')
    
    def system_command_callback(self, msg):
        command = msg.data.lower()
        
        if command == 'shutdown':
            self.get_logger().info('Shutdown command received')
            self.publish_status_message('shutting_down')
            rclpy.shutdown()
        
        elif command == 'restart':
            self.get_logger().info('Restart command received')
            self.publish_status_message('restarting')
        
        elif command == 'status':
            self.publish_status()
        
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def publish_status(self):
        status = {
            'system': 'online',
            'mode': 'operational',
            'nodes': 'all_running',
            'timestamp': self.get_clock().now().to_msg()
        }
        
        self.publish_status_message(f'online_operational_all_running')
    
    def publish_status_message(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HexapodBringup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
