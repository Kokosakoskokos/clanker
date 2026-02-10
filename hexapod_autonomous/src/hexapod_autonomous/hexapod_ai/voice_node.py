#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import pyttsx3
import threading
import queue
import yaml
import os


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        
        self.get_logger().info('Initializing Voice Node...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/ai_config.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.voice_command_pub = self.create_publisher(
            String,
            '/voice_command',
            10
        )
        
        self.response_sub = self.create_subscription(
            String,
            '/ai_response',
            self.response_callback,
            10
        )
        
        self.recognizer = sr.Recognizer()
        self.engine = pyttsx3.init()
        
        self.setup_voice_engine()
        
        self.is_listening = False
        self.command_queue = queue.Queue()
        
        self.listen_thread = threading.Thread(
            target=self.listen_loop,
            daemon=True
        )
        self.listen_thread.start()
        
        self.process_timer = self.create_timer(
            0.1,
            self.process_commands
        )
        
        self.get_logger().info('Voice Node initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'voice_rate': 150,
            'voice_volume': 0.8,
            'language': 'en-US',
            'keyword': 'robot',
            'wake_word_timeout': 3.0
        }
    
    def setup_voice_engine(self):
        voice_config = self.config.get('voice', {})
        
        voices = self.engine.getProperty('voices')
        
        czech_voice = None
        for voice in voices:
            if 'czech' in voice.name.lower() or 'cs' in voice.languages:
                czech_voice = voice
                break
        
        if czech_voice:
            self.engine.setProperty('voice', czech_voice.id)
            self.get_logger().info(f'Using Czech voice: {czech_voice.name}')
        elif len(voices) > 0:
            self.engine.setProperty('voice', voices[0].id)
        
        self.engine.setProperty(
            'rate',
            voice_config.get('voice_rate', 150)
        )
        self.engine.setProperty(
            'volume',
            voice_config.get('voice_volume', 0.8)
        )
    
    def listen_loop(self):
        with sr.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            self.get_logger().info('Microphone calibrated')
            
            while rclpy.ok():
                try:
                    self.listen_for_command(source)
                except Exception as e:
                    self.get_logger().error(f'Error in listen loop: {e}')
                    import time
                    time.sleep(1)
    
    def listen_for_command(self, source):
        try:
            audio = self.recognizer.listen(source, timeout=5)
            
            try:
                text = self.recognizer.recognize_google(
                    audio,
                    language=self.config.get('voice', {}).get('language', 'cs-CZ')
                )
                
                self.get_logger().info(f'Heard: {text}')
                
                keyword = self.config.get('voice', {}).get('keyword', 'robot')
                
                if keyword.lower() in text.lower():
                    command = text.lower().replace(keyword, '').strip()
                    if command:
                        self.command_queue.put(command)
                        czech_phrases = self.config.get('voice', {}).get('czech_phrases', {})
                        self.speak(czech_phrases.get('following', 'Slyším: ' + command))
                
            except sr.UnknownValueError:
                pass
            except sr.RequestError as e:
                self.get_logger().error(f'Speech recognition error: {e}')
                
        except sr.WaitTimeoutError:
            pass
        except Exception as e:
            self.get_logger().error(f'Error listening: {e}')
    
    def process_commands(self):
        while not self.command_queue.empty():
            try:
                command = self.command_queue.get_nowait()
                msg = String()
                msg.data = command
                self.voice_command_pub.publish(msg)
            except queue.Empty:
                break
    
    def response_callback(self, msg):
        self.speak(msg.data)
    
    def speak(self, text):
        try:
            self.engine.say(text)
            self.engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f'Error speaking: {e}')
    
    def destroy_node(self):
        self.is_listening = False
        if hasattr(self, 'listen_thread'):
            self.listen_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
