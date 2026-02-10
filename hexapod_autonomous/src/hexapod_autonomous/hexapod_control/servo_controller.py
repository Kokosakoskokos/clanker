#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import board
import busio
from adafruit_servokit import ServoKit
import time
import yaml
import os


class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        self.get_logger().info('Initializing Servo Controller...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/servos.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.kits = []
        self.init_pca9685_controllers()
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.servo_cmd_sub = self.create_subscription(
            String,
            '/servo_command',
            self.servo_command_callback,
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/servo_status',
            10
        )
        
        self.current_positions = {}
        self.init_servo_positions()
        
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Servo Controller initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'pca9685': [
                {'i2c_address': 0x40, 'channels': 16},
                {'i2c_address': 0x41, 'channels': 16}
            ],
            'servos': {
                'front_right': {'coxa': 0, 'femur': 1, 'tibia': 2},
                'middle_right': {'coxa': 4, 'femur': 5, 'tibia': 6},
                'rear_right': {'coxa': 8, 'femur': 9, 'tibia': 10},
                'front_left': {'coxa': 3, 'femur': 15, 'tibia': 14},
                'middle_left': {'coxa': 7, 'femur': 12, 'tibia': 13},
                'rear_left': {'coxa': 11, 'femur': 8, 'tibia': 9}
            },
            'neutral_positions': {
                'coxa': 90,
                'femur': 90,
                'tibia': 90
            }
        }
    
    def init_pca9685_controllers(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            
            for pca_config in self.config['pca9685']:
                kit = ServoKit(
                    channels=pca_config['channels'],
                    i2c=i2c,
                    address=pca_config['i2c_address']
                )
                self.kits.append(kit)
                self.get_logger().info(
                    f'PCA9685 controller initialized at 0x{pca_config["i2c_address"]:02x}'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCA9685: {e}')
            self.get_logger().warn('Running in simulation mode')
    
    def init_servo_positions(self):
        for leg_name, servos in self.config['servos'].items():
            for joint, channel in servos.items():
                neutral = self.config['neutral_positions'][joint]
                self.set_servo_angle(channel, neutral)
                self.current_positions[channel] = neutral
        
        time.sleep(0.5)
    
    def set_servo_angle(self, channel, angle):
        try:
            kit_index = 0 if channel < 16 else 1
            kit_channel = channel if channel < 16 else channel - 16
            
            self.kits[kit_index].servo[kit_channel].angle = angle
            self.current_positions[channel] = angle
            return True
        except Exception as e:
            self.get_logger().error(f'Error setting servo {channel}: {e}')
            return False
    
    def cmd_vel_callback(self, msg):
        self.get_logger().debug(
            f'Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}'
        )
        
    def servo_command_callback(self, msg):
        try:
            parts = msg.data.split()
            if len(parts) >= 2:
                channel = int(parts[0])
                angle = float(parts[1])
                self.set_servo_angle(channel, angle)
                self.get_logger().info(f'Set servo {channel} to {angle} degrees')
        except Exception as e:
            self.get_logger().error(f'Error processing servo command: {e}')
    
    def publish_status(self):
        status = {
            'active': True,
            'servo_count': len(self.current_positions),
            'controllers': len(self.kits)
        }
        msg = String()
        msg.data = str(status)
        self.status_pub.publish(msg)
    
    def emergency_stop(self):
        for channel, pos in self.current_positions.items():
            self.set_servo_angle(channel, 90)
        self.get_logger().warn('Emergency stop activated')


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
