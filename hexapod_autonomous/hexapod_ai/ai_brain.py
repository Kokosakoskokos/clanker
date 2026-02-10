#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from openai import OpenAI
import yaml
import os


class AIBrain(Node):
    def __init__(self):
        super().__init__('ai_brain')
        
        self.get_logger().info('Initializing AI Brain...')
        
        config_path = os.path.join(
            os.path.dirname(__file__),
            '../../../config/ai_config.yaml'
        )
        self.config = self.load_config(config_path)
        
        self.client = self.setup_openai()
        
        self.voice_sub = self.create_subscription(
            String,
            '/voice_command',
            self.voice_command_callback,
            10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )
        
        self.nav_cmd_pub = self.create_publisher(
            String,
            '/navigation_command',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.gait_cmd_pub = self.create_publisher(
            String,
            '/gait_mode',
            10
        )
        
        self.response_pub = self.create_publisher(
            String,
            '/ai_response',
            10
        )
        
        self.current_detections = []
        self.context = {
            'mode': 'autonomous',
            'last_action': 'none',
            'detected_objects': []
        }
        
        self.get_logger().info('AI Brain initialized')
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}')
            return self.get_default_config()
    
    def get_default_config(self):
        return {
            'model': 'gpt-4',
            'temperature': 0.7,
            'max_tokens': 150
        }
    
    def setup_openai(self):
        ai_config = self.config.get('ai', {})
        llm_config = ai_config.get('llm', {})
        provider = llm_config.get('provider', 'openrouter')
        
        if provider == 'openrouter':
            api_key = os.getenv('OPENROUTER_API_KEY')
            if not api_key:
                self.get_logger().warn('OPENROUTER_API_KEY not set, using simulation mode')
                return None
            
            openrouter_config = self.config.get('openrouter', {})
            client = OpenAI(
                api_key=api_key,
                base_url=llm_config.get('base_url', 'https://openrouter.ai/api/v1'),
                default_headers={
                    "HTTP-Referer": openrouter_config.get('site_url', ''),
                    "X-Title": openrouter_config.get('app_name', 'Hexapod Robot')
                }
            )
            self.get_logger().info('OpenRouter client initialized')
            return client
        else:
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                self.get_logger().warn('API key not set, using simulation mode')
                return None
            
            client = OpenAI(api_key=api_key)
            self.get_logger().info('OpenAI client initialized')
            return client
    
    def voice_command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')
        
        if self.client:
            response = self.process_with_ai(command)
        else:
            response = self.process_command_locally(command)
        
        self.response_pub.publish(String(data=response))
    
    def detection_callback(self, msg):
        self.current_detections = []
        for detection in msg.detections:
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
                self.current_detections.append({
                    'class': class_id,
                    'confidence': score
                })
        
        self.context['detected_objects'] = self.current_detections
    
    def process_with_ai(self, command):
        try:
            prompt = self.build_prompt(command)
            
            ai_config = self.config.get('ai', {})
            llm_config = ai_config.get('llm', {})
            
            response = self.client.chat.completions.create(
                model=llm_config.get('model', 'mistralai/mistral-7b-instruct:free'),
                messages=[
                    {
                        'role': 'system',
                        'content': 'You are an autonomous hexapod robot assistant. '
                                 'Respond in Czech language. '
                                 'Respond with one of these commands: '
                                 'zastavit, autonomní, dopředu X, dozadu X, '
                                 'vlevo X, vpravo X, nebo přirozenou odpověď. '
                                 'X je rychlost mezi 0.1 a 0.5. '
                                 'Pro sledování lidí použij: sledovat jméno.'
                    },
                    {
                        'role': 'user',
                        'content': prompt
                    }
                ],
                temperature=llm_config.get('temperature', 0.7),
                max_tokens=llm_config.get('max_tokens', 150)
            )
            
            ai_response = response.choices[0].message.content
            self.execute_ai_response(ai_response)
            
            return ai_response
            
        except Exception as e:
            self.get_logger().error(f'AI processing error: {e}')
            return self.process_command_locally(command)
    
    def build_prompt(self, command):
        prompt = f'Current mode: {self.context["mode"]}\n'
        prompt += f'Last action: {self.context["last_action"]}\n'
        
        if self.current_detections:
            objects = [d['class'] for d in self.current_detections]
            prompt += f'Detected objects: {", ".join(objects)}\n'
        else:
            prompt += 'No objects detected\n'
        
        prompt += f'Command: {command}\n'
        prompt += 'What should I do?'
        
        return prompt
    
    def execute_ai_response(self, response):
        response_lower = response.lower().strip()
        
        czech_stop = ['zastavit', 'zastav', 'stop']
        czech_autonomous = ['autonomní', 'automaticky', 'autonomní mód']
        czech_forward = ['dopředu', 'vpřed', 'dopředu']
        czech_backward = ['dozadu', 'zpátky', 'dozadu']
        czech_left = ['vlevo', 'doleva', 'vlevo']
        czech_right = ['vpravo', 'doprava', 'vpravo']
        czech_follow = ['sledovat', 'sleduj']
        
        if any(word in response_lower for word in czech_stop):
            self.nav_cmd_pub.publish(String(data='stop'))
            self.context['mode'] = 'stopped'
            self.context['last_action'] = 'stopped'
        
        elif any(word in response_lower for word in czech_autonomous):
            self.nav_cmd_pub.publish(String(data='autonomous'))
            self.context['mode'] = 'autonomous'
            self.context['last_action'] = 'autonomous'
        
        elif any(word in response_lower for word in czech_forward):
            speed = self.extract_speed(response_lower, 0.2)
            self.nav_cmd_pub.publish(String(data=f'forward {speed}'))
            self.context['mode'] = 'manual'
            self.context['last_action'] = f'forward {speed}'
        
        elif any(word in response_lower for word in czech_backward):
            speed = self.extract_speed(response_lower, 0.2)
            self.nav_cmd_pub.publish(String(data=f'backward {speed}'))
            self.context['mode'] = 'manual'
            self.context['last_action'] = f'backward {speed}'
        
        elif any(word in response_lower for word in czech_left):
            speed = self.extract_speed(response_lower, 0.3)
            self.nav_cmd_pub.publish(String(data=f'left {speed}'))
            self.context['mode'] = 'manual'
            self.context['last_action'] = f'left {speed}'
        
        elif any(word in response_lower for word in czech_right):
            speed = self.extract_speed(response_lower, 0.3)
            self.nav_cmd_pub.publish(String(data=f'right {speed}'))
            self.context['mode'] = 'manual'
            self.context['last_action'] = f'right {speed}'
        
        elif any(word in response_lower for word in czech_follow):
            import re
            match = re.search(r'sledovat\s+(\w+)', response_lower)
            if match:
                name = match.group(1)
                self.nav_cmd_pub.publish(String(data=f'follow {name}'))
                self.context['mode'] = 'following'
                self.context['last_action'] = f'following {name}'
        
        elif 'tripod' in response_lower or 'třínohá' in response_lower:
            self.gait_cmd_pub.publish(String(data='tripod'))
            self.context['last_action'] = 'gait tripod'
        
        elif 'wave' in response_lower or 'vlna' in response_lower:
            self.gait_cmd_pub.publish(String(data='wave'))
            self.context['last_action'] = 'gait wave'
            self.context['last_action'] = 'gait wave'
    
    def extract_speed(self, text, default_speed):
        try:
            import re
            match = re.search(r'(\d+\.?\d*)', text)
            if match:
                speed = float(match.group(1))
                return max(0.1, min(speed, 0.5))
        except:
            pass
        return default_speed
    
    def process_command_locally(self, command):
        command_lower = command.lower()
        
        czech_stop = ['zastavit', 'zastav', 'stop']
        czech_autonomous = ['autonomní', 'automaticky', 'start']
        czech_forward = ['dopředu', 'vpřed', 'dopředu']
        czech_backward = ['dozadu', 'zpátky', 'dozadu']
        czech_left = ['vlevo', 'doleva', 'vlevo']
        czech_right = ['vpravo', 'doprava', 'vpravo']
        czech_follow = ['sledovat', 'sleduj']
        czech_see = ['co vidíš', 'co vidis', 'co vidíte', 'vidíš']
        
        if any(word in command_lower for word in czech_stop):
            self.nav_cmd_pub.publish(String(data='stop'))
            self.context['mode'] = 'stopped'
            return 'Zastavuji se.'
        
        elif any(word in command_lower for word in czech_autonomous):
            self.nav_cmd_pub.publish(String(data='autonomous'))
            self.context['mode'] = 'autonomous'
            return 'Spouštím autonomní mód.'
        
        elif any(word in command_lower for word in czech_forward):
            speed = self.extract_speed(command_lower, 0.2)
            self.nav_cmd_pub.publish(String(data=f'forward {speed}'))
            return f'Hýbu se dopředu rychlostí {speed}.'
        
        elif any(word in command_lower for word in czech_backward):
            speed = self.extract_speed(command_lower, 0.2)
            self.nav_cmd_pub.publish(String(data=f'backward {speed}'))
            return f'Hýbu se dozadu rychlostí {speed}.'
        
        elif any(word in command_lower for word in czech_left):
            speed = self.extract_speed(command_lower, 0.3)
            self.nav_cmd_pub.publish(String(data=f'left {speed}'))
            return f'Otočím se vlevo rychlostí {speed}.'
        
        elif any(word in command_lower for word in czech_right):
            speed = self.extract_speed(command_lower, 0.3)
            self.nav_cmd_pub.publish(String(data=f'right {speed}'))
            return f'Otočím se vpravo rychlostí {speed}.'
        
        elif any(word in command_lower for word in czech_see):
            if self.current_detections:
                objects = [d['class'] for d in self.current_detections]
                return f'Vidím: {", ".join(objects)}.'
            else:
                return 'Nic momentálně nevidím.'
        
        else:
            return 'Nerozumím tomu příkazu. Zkuste říct: start, zastavit, hýb dopředu, otoč vlevo, nebo co vidíš.'


def main(args=None):
    rclpy.init(args=args)
    node = AIBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
