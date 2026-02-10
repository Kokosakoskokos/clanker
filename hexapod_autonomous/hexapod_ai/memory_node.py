#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import pickle
import json
import yaml
from datetime import datetime


class MemoryNode(Node):
    def __init__(self):
        super().__init__('memory_node')
        
        self.get_logger().info('Initializing Memory Node...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/navigation.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.data_dir = os.path.join(
            os.path.dirname(__file__),
            '../../../data'
        )
        os.makedirs(self.data_dir, exist_ok=True)
        
        self.command_sub = self.create_subscription(
            String,
            '/memory_command',
            self.memory_command_callback,
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/memory_status',
            10
        )
        
        self.faces_db = self.load_database('faces_db.json')
        self.interactions_db = self.load_database('interactions_db.json')
        self.preferences_db = self.load_database('preferences_db.json')
        
        self.current_interaction = None
        
        self.get_logger().info('Memory Node initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return {}
    
    def load_database(self, filename):
        filepath = os.path.join(self.data_dir, filename)
        
        if os.path.exists(filepath):
            try:
                with open(filepath, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().error(f'Error loading {filename}: {e}')
                return {}
        
        return {}
    
    def save_database(self, data, filename):
        filepath = os.path.join(self.data_dir, filename)
        
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            return True
        except Exception as e:
            self.get_logger().error(f'Error saving {filename}: {e}')
            return False
    
    def memory_command_callback(self, msg):
        parts = msg.data.split()
        
        if len(parts) < 1:
            return
        
        command = parts[0].lower()
        
        if command == 'remember_face':
            if len(parts) >= 2:
                name = ' '.join(parts[1:])
                self.remember_face(name)
        
        elif command == 'forget_face':
            if len(parts) >= 2:
                name = ' '.join(parts[1:])
                self.forget_face(name)
        
        elif command == 'list_faces':
            self.list_faces()
        
        elif command == 'get_face_info':
            if len(parts) >= 2:
                name = ' '.join(parts[1:])
                self.get_face_info(name)
        
        elif command == 'start_interaction':
            if len(parts) >= 2:
                name = ' '.join(parts[1:])
                self.start_interaction(name)
        
        elif command == 'end_interaction':
            self.end_interaction()
        
        elif command == 'set_preference':
            if len(parts) >= 3:
                key = parts[1]
                value = ' '.join(parts[2:])
                self.set_preference(key, value)
        
        elif command == 'get_preference':
            if len(parts) >= 2:
                key = parts[1]
                self.get_preference(key)
        
        elif command == 'note':
            if len(parts) >= 2:
                note = ' '.join(parts[1:])
                self.add_note(note)
        
        elif command == 'get_notes':
            self.get_notes()
    
    def remember_face(self, name):
        if name in self.faces_db:
            self.get_logger().info(f'Face already exists: {name}')
            self.publish_status(f'exists_{name}')
            return
        
        self.faces_db[name] = {
            'name': name,
            'first_seen': datetime.now().isoformat(),
            'last_seen': datetime.now().isoformat(),
            'encounter_count': 1,
            'notes': [],
            'preferences': {}
        }
        
        self.save_database(self.faces_db, 'faces_db.json')
        self.get_logger().info(f'Remembered face: {name}')
        self.publish_status(f'remembered_{name}')
    
    def forget_face(self, name):
        if name in self.faces_db:
            del self.faces_db[name]
            self.save_database(self.faces_db, 'faces_db.json')
            self.get_logger().info(f'Forgot face: {name}')
            self.publish_status(f'forgotten_{name}')
        else:
            self.get_logger().warn(f'Face not found: {name}')
            self.publish_status(f'not_found_{name}')
    
    def list_faces(self):
        if self.faces_db:
            names = ', '.join(self.faces_db.keys())
            self.get_logger().info(f'Known faces: {names}')
            self.publish_status(f'faces_{names}')
        else:
            self.get_logger().info('No known faces')
            self.publish_status('no_faces')
    
    def get_face_info(self, name):
        if name in self.faces_db:
            info = self.faces_db[name]
            info_str = f"{name}: seen {info['encounter_count']} times, first {info['first_seen']}"
            self.get_logger().info(info_str)
            self.publish_status(f'info_{info_str}')
        else:
            self.get_logger().warn(f'Face not found: {name}')
            self.publish_status(f'not_found_{name}')
    
    def start_interaction(self, name):
        self.current_interaction = {
            'name': name,
            'start_time': datetime.now().isoformat(),
            'notes': []
        }
        
        if name in self.faces_db:
            self.faces_db[name]['last_seen'] = datetime.now().isoformat()
            self.faces_db[name]['encounter_count'] += 1
            self.save_database(self.faces_db, 'faces_db.json')
        
        self.get_logger().info(f'Started interaction with: {name}')
        self.publish_status(f'interaction_started_{name}')
    
    def end_interaction(self):
        if self.current_interaction:
            name = self.current_interaction['name']
            end_time = datetime.now().isoformat()
            
            if name not in self.interactions_db:
                self.interactions_db[name] = []
            
            self.interactions_db[name].append({
                'start': self.current_interaction['start_time'],
                'end': end_time,
                'notes': self.current_interaction['notes']
            })
            
            self.save_database(self.interactions_db, 'interactions_db.json')
            self.get_logger().info(f'Ended interaction with: {name}')
            self.publish_status(f'interaction_ended_{name}')
            
            self.current_interaction = None
        else:
            self.get_logger().warn('No active interaction')
            self.publish_status('no_interaction')
    
    def set_preference(self, key, value):
        if self.current_interaction:
            name = self.current_interaction['name']
            
            if name in self.faces_db:
                self.faces_db[name]['preferences'][key] = value
                self.save_database(self.faces_db, 'faces_db.json')
                self.get_logger().info(f'Set preference for {name}: {key}={value}')
                self.publish_status(f'preference_set_{key}')
            else:
                self.get_logger().warn(f'Face not found: {name}')
                self.publish_status(f'not_found_{name}')
        else:
            self.get_logger().warn('No active interaction')
            self.publish_status('no_interaction')
    
    def get_preference(self, key):
        if self.current_interaction:
            name = self.current_interaction['name']
            
            if name in self.faces_db:
                preferences = self.faces_db[name]['preferences']
                value = preferences.get(key, 'Not set')
                self.get_logger().info(f'Preference for {name}: {key}={value}')
                self.publish_status(f'preference_{key}_{value}')
            else:
                self.get_logger().warn(f'Face not found: {name}')
                self.publish_status(f'not_found_{name}')
        else:
            self.get_logger().warn('No active interaction')
            self.publish_status('no_interaction')
    
    def add_note(self, note):
        if self.current_interaction:
            self.current_interaction['notes'].append({
                'time': datetime.now().isoformat(),
                'content': note
            })
            self.get_logger().info(f'Added note: {note}')
            self.publish_status(f'note_added')
        else:
            self.get_logger().warn('No active interaction')
            self.publish_status('no_interaction')
    
    def get_notes(self):
        if self.current_interaction:
            notes = self.current_interaction['notes']
            if notes:
                note_texts = [n['content'] for n in notes]
                notes_str = '; '.join(note_texts)
                self.get_logger().info(f'Notes: {notes_str}')
                self.publish_status(f'notes_{notes_str}')
            else:
                self.get_logger().info('No notes')
                self.publish_status('no_notes')
        else:
            self.get_logger().warn('No active interaction')
            self.publish_status('no_interaction')
    
    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.current_interaction:
            node.end_interaction()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
