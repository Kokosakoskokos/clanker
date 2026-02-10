#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
import yaml
import os


class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        
        self.get_logger().info('Initializing Ultrasonic Node...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/navigation.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.publishers = {}
        self.init_sensors()
        
        self.timer = self.create_timer(
            0.1,
            self.read_sensors
        )
        
        self.get_logger().info('Ultrasonic Node initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'sensors': [
                {'name': 'front', 'trig': 23, 'echo': 24, 'frame_id': 'ultrasonic_front'},
                {'name': 'left', 'trig': 17, 'echo': 27, 'frame_id': 'ultrasonic_left'},
                {'name': 'right', 'trig': 5, 'echo': 6, 'frame_id': 'ultrasonic_right'},
                {'name': 'rear', 'trig': 22, 'echo': 10, 'frame_id': 'ultrasonic_rear'}
            ],
            'max_range': 4.0,
            'min_range': 0.02,
            'field_of_view': 0.26
        }
    
    def init_sensors(self):
        try:
            GPIO.setmode(GPIO.BCM)
            
            for sensor in self.config['sensors']:
                GPIO.setup(sensor['trig'], GPIO.OUT)
                GPIO.setup(sensor['echo'], GPIO.IN)
                
                topic = f'/{sensor["name"]}_distance'
                self.publishers[sensor['name']] = self.create_publisher(
                    Range,
                    topic,
                    10
                )
                
                self.get_logger().info(f'Sensor {sensor["name"]} initialized on GPIO {sensor["trig"]}/{sensor["echo"]}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to initialize GPIO: {e}')
            self.get_logger().warn('Running in simulation mode')
    
    def read_sensors(self):
        for sensor in self.config['sensors']:
            try:
                distance = self.measure_distance(sensor['trig'], sensor['echo'])
                self.publish_range(sensor, distance)
            except Exception as e:
                self.get_logger().error(f'Error reading sensor {sensor["name"]}: {e}')
                self.publish_simulation_range(sensor)
    
    def measure_distance(self, trig_pin, echo_pin):
        GPIO.output(trig_pin, False)
        time.sleep(0.02)
        
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)
        
        start_time = time.time()
        timeout = start_time + 0.1
        
        while GPIO.input(echo_pin) == 0 and start_time < timeout:
            start_time = time.time()
        
        stop_time = time.time()
        timeout = stop_time + 0.1
        
        while GPIO.input(echo_pin) == 1 and stop_time < timeout:
            stop_time = time.time()
        
        time_elapsed = stop_time - start_time
        
        distance = (time_elapsed * 34300) / 2
        
        return max(self.config['min_range'], 
                  min(distance, self.config['max_range']))
    
    def publish_range(self, sensor, distance):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = sensor['frame_id']
        
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.config['field_of_view']
        msg.min_range = self.config['min_range']
        msg.max_range = self.config['max_range']
        msg.range = distance
        
        self.publishers[sensor['name']].publish(msg)
    
    def publish_simulation_range(self, sensor):
        import random
        distance = random.uniform(0.3, 2.0)
        self.publish_range(sensor, distance)
    
    def destroy_node(self):
        try:
            GPIO.cleanup()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
