#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import yaml
import os
import time
import math


class GaitController(Node):
    def __init__(self):
        super().__init__('gait_controller')
        
        self.get_logger().info('Initializing Gait Controller...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/gait_params.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.gait_mode_sub = self.create_subscription(
            String,
            '/gait_mode',
            self.gait_mode_callback,
            10
        )
        
        self.servo_cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/servo_commands',
            10
        )
        
        self.gait_mode = 'tripod'
        self.current_step = 0
        self.last_cmd_time = time.time()
        self.is_moving = False
        
        self.gait_patterns = self.init_gait_patterns()
        self.init_leg_positions()
        
        self.gait_timer = self.create_timer(
            self.config['step_period'],
            self.gait_step
        )
        
        self.get_logger().info('Gait Controller initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'step_period': 0.2,
            'stride_length': 30,
            'step_height': 20,
            'leg_length': 100
        }
    
    def init_gait_patterns(self):
        patterns = {
            'tripod': {
                'group1': ['front_right', 'rear_left', 'middle_left'],
                'group2': ['front_left', 'rear_right', 'middle_right'],
                'sequence': ['lift', 'forward', 'lower', 'back']
            },
            'wave': {
                'sequence': ['front_left', 'middle_left', 'rear_left',
                           'rear_right', 'middle_right', 'front_right']
            },
            'ripple': {
                'group1': ['front_left', 'rear_right'],
                'group2': ['middle_left', 'middle_right'],
                'group3': ['rear_left', 'front_right']
            }
        }
        return patterns
    
    def init_leg_positions(self):
        self.leg_positions = {
            'front_right': [90, 90, 90],
            'middle_right': [90, 90, 90],
            'rear_right': [90, 90, 90],
            'front_left': [90, 90, 90],
            'middle_left': [90, 90, 90],
            'rear_left': [90, 90, 90]
        }
        
        self.servo_mapping = {
            'front_right': [0, 1, 2],
            'middle_right': [4, 5, 6],
            'rear_right': [8, 9, 10],
            'front_left': [3, 15, 14],
            'middle_left': [7, 12, 13],
            'rear_left': [11, 8, 9]
        }
    
    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()
        self.is_moving = True
        self.current_velocity = msg
    
    def gait_mode_callback(self, msg):
        self.gait_mode = msg.data
        self.get_logger().info(f'Gait mode changed to: {self.gait_mode}')
    
    def gait_step(self):
        if time.time() - self.last_cmd_time > 1.0:
            self.is_moving = False
            return
        
        if not self.is_moving:
            return
        
        if self.gait_mode == 'tripod':
            self.tripod_gait_step()
        elif self.gait_mode == 'wave':
            self.wave_gait_step()
        elif self.gait_mode == 'ripple':
            self.ripple_gait_step()
        
        self.publish_servo_commands()
        self.current_step = (self.current_step + 1) % 4
    
    def tripod_gait_step(self):
        pattern = self.gait_patterns['tripod']
        
        linear_x = getattr(self, 'current_velocity', None)
        if linear_x is None:
            return
        
        stride = self.config['stride_length'] * linear_x.linear_x
        height = self.config['step_height']
        
        phase = self.current_step
        
        for leg in pattern['group1']:
            self.apply_leg_movement(leg, stride, height, phase)
        
        for leg in pattern['group2']:
            self.apply_leg_movement(leg, -stride, height, (phase + 2) % 4)
    
    def apply_leg_movement(self, leg, stride, height, phase):
        if leg not in self.leg_positions:
            return
        
        coxa, femur, tibia = self.leg_positions[leg]
        
        if phase == 0:
            femur = 90 + height * 0.3
            tibia = 90 + height * 0.5
        elif phase == 1:
            coxa = 90 + stride * 0.5
            femur = 90 + height * 0.5
        elif phase == 2:
            femur = 90 - height * 0.3
            tibia = 90 - height * 0.5
        elif phase == 3:
            coxa = 90 - stride * 0.5
            femur = 90 - height * 0.5
        
        self.leg_positions[leg] = [
            np.clip(coxa, 30, 150),
            np.clip(femur, 30, 150),
            np.clip(tibia, 30, 150)
        ]
    
    def wave_gait_step(self):
        pattern = self.gait_patterns['wave']
        leg_idx = self.current_step % len(pattern['sequence'])
        leg = pattern['sequence'][leg_idx]
        
        self.apply_leg_movement(leg, 20, 15, 0)
    
    def ripple_gait_step(self):
        pattern = self.gait_patterns['ripple']
        group_idx = self.current_step % 3
        
        if group_idx == 0:
            for leg in pattern['group1']:
                self.apply_leg_movement(leg, 20, 15, 0)
        elif group_idx == 1:
            for leg in pattern['group2']:
                self.apply_leg_movement(leg, 20, 15, 0)
        else:
            for leg in pattern['group3']:
                self.apply_leg_movement(leg, 20, 15, 0)
    
    def publish_servo_commands(self):
        commands = Float32MultiArray()
        
        for leg_name, servos in self.servo_mapping.items():
            for i, channel in enumerate(servos):
                commands.data.append(self.leg_positions[leg_name][i])
        
        self.servo_cmd_pub.publish(commands)
    
    def emergency_stop(self):
        for leg in self.leg_positions:
            self.leg_positions[leg] = [90, 90, 90]
        self.publish_servo_commands()
        self.get_logger().warn('Emergency stop activated')


def main(args=None):
    rclpy.init(args=args)
    node = GaitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.emergency_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
