# Autonomous Hexapod Robot System

A ROS2-based autonomous hexapod robot with face recognition, person following, voice control, and AI-powered navigation.

## Features

- **Face Recognition**: Detect and recognize people using computer vision
- **Person Following**: Automatically follow recognized individuals
- **Memory System**: Remember people, preferences, and interactions
- **Voice Commands**: Natural language voice control with speech recognition
- **AI-Powered Decision Making**: LLM integration for intelligent command interpretation
- **Computer Vision**: Real-time object detection using YOLO and face recognition
- **Multi-Sensor Integration**: Camera and ultrasonic sensors (LiDAR-free design)
- **Hexapod Locomotion**: Multiple gait patterns (tripod, wave, ripple)
- **Servo Control**: Dual PCA9685 controllers for 18 MG995 servo motors
- **Modular Architecture**: ROS2-based design for easy maintenance and upgrades

## Hardware Requirements

Based on 3D-Printed Hexapod (Instructables):
- Raspberry Pi 4 (8GB recommended)
- 18x MG995 servo motors
- 2x PCA9685 servo controllers
- USB Camera
- 4x HC-SR04 ultrasonic sensors
- 3.7V LiPo battery pack (2S or 3S)
- I2C connections for PCA9685

## Software Requirements

- ROS2 Humble (or Foxy)
- Python 3.8+
- Ubuntu 22.04 LTS (recommended)

## Installation

### 1. Install ROS2

Follow the official ROS2 installation guide for Ubuntu:

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Install Python Dependencies

```bash
cd hexapod_autonomous
pip3 install -r requirements.txt
```

### 3. Enable I2C and Camera

```bash
sudo raspi-config
```

Navigate to:
- Interface Options -> I2C -> Enable
- Interface Options -> Camera -> Enable

### 4. Setup OpenRouter API Key (Free Models)

For AI-powered voice commands, set your OpenRouter API key:

```bash
export OPENROUTER_API_KEY="your_api_key_here"
```

Add to `.bashrc` for persistence:

```bash
echo 'export OPENROUTER_API_KEY="your_api_key_here"' >> ~/.bashrc
source ~/.bashrc
```

The system uses **Mistral 7B Instruct (free)** via OpenRouter, giving you 1000 requests for $10 USD.

### 5. Build the Package

```bash
cd ..
colcon build --packages-select hexapod_autonomous
source install/setup.bash
```

## Usage

### Launch All Nodes

```bash
ros2 launch hexapod_autonomous hexapod_full.launch.py
```

### Launch Individual Subsystems

Bringup only:
```bash
ros2 launch hexapod_autonomous hexapod_bringup.launch.py
```

Control only:
```bash
ros2 launch hexapod_autonomous hexapod_control.launch.py
```

Vision only:
```bash
ros2 launch hexapod_autonomous hexapod_vision.launch.py
```

Navigation only:
```bash
ros2 launch hexapod_autonomous hexapod_navigation.launch.py
```

AI and Voice only:
```bash
ros2 launch hexapod_autonomous hexapod_ai.launch.py
```

## Voice Commands (Czech)

The robot responds to natural language commands in Czech. Say "robot" followed by your command:

Movement:
- "robot, start" - Start autonomous mode
- "robot, zastavit" - Stop all movement
- "robot, hýb dopředu" - Move forward
- "robot, hýb dozadu" - Move backward
- "robot, otoč vlevo" - Turn left
- "robot, otoč vpravo" - Turn right
- "robot, sledovat [jméno]" - Follow a specific person

Information:
- "robot, co vidíš" - List detected objects
- "robot, kdo to je" - Ask about detected people

Gait Control:
- "robot, tripod chůze" - Switch to tripod gait
- "robot, vlnová chůze" - Switch to wave gait

## Face Recognition & Memory

The robot can remember people and their faces:

Remember a person:
```bash
ros2 topic echo /face_command
# Then publish: "register John"
```

List known faces:
```bash
ros2 topic pub /face_command std_msgs/String "data: 'list_faces'"
```

Forget a person:
```bash
ros2 topic pub /face_command std_msgs/String "data: 'forget John'"
```

Start/stop following:
```bash
ros2 topic pub /person_follow_command std_msgs/String "data: 'follow John'"
ros2 topic pub /person_follow_command std_msgs/String "data: 'stop'"
```

## Configuration

Edit configuration files in the `config/` directory:

- `servos.yaml` - Servo pin assignments and MG995 calibration
- `gait_params.yaml` - Gait patterns and parameters
- `camera.yaml` - Camera, face recognition, and object detection settings
- `navigation.yaml` - Sensor, person following, and face recognition parameters
- `ai_config.yaml` - OpenRouter, voice, and Czech language settings

## Czech Language Support

The robot speaks and understands Czech:

**Text-to-Speech:**
- Uses eSpeak for Czech voice synthesis
- Automatic Czech voice detection
- Configurable phrases in `config/ai_config.yaml`

**Speech Recognition:**
- Google Speech Recognition with Czech language (cs-CZ)
- Wake word: "robot"
- Supports natural Czech commands

**AI Responses:**
- OpenRouter with Mistral 7B Instruct
- All AI responses in Czech
- Context-aware Czech responses

## Project Structure

```
hexapod_autonomous/
├── config/                 # Configuration files
├── launch/                 # Launch scripts
├── src/
│   └── hexapod_autonomous/
│       ├── hexapod_bringup/     # System initialization
│       ├── hexapod_control/     # Servo and gait control
│       ├── hexapod_vision/     # Camera, object detection, face recognition
│       ├── hexapod_perception/ # Ultrasonic sensors
│       ├── hexapod_navigation/ # Navigation, obstacle avoidance, person following
│       └── hexapod_ai/          # Voice, AI processing, memory system
├── data/                   # Face database and interactions
├── models/                 # ML models (YOLO, face recognition)
├── requirements.txt        # Python dependencies
├── package.xml            # ROS2 package metadata
└── setup.py               # Package setup
```

## ROS2 Topics

### Published Topics

- `/camera/image_raw` - Camera images
- `/detections` - Object detections (YOLO)
- `/face_detections` - Face detections
- `/recognized_faces` - Recognized person names
- `/*_distance` - Ultrasonic sensor readings (front, left, right, rear)
- `/servo_status` - Servo controller status
- `/system_status` - System status
- `/ai_response` - AI verbal responses (Czech)
- `/follow_status` - Person following status
- `/memory_status` - Memory system status

### Subscribed Topics

- `/cmd_vel` - Velocity commands
- `/gait_mode` - Gait pattern selection
- `/servo_command` - Individual servo commands
- `/voice_command` - Voice commands (Czech)
- `/navigation_command` - Navigation commands
- `/face_command` - Face recognition commands
- `/person_follow_command` - Person following commands
- `/memory_command` - Memory system commands

## Troubleshooting

### OpenRouter API Issues

Test API key:
```bash
curl -H "Authorization: Bearer $OPENROUTER_API_KEY" \
     https://openrouter.ai/api/v1/models
```

Check available free models on https://openrouter.ai/models?free=true

### I2C Communication Issues

Check I2C devices:
```bash
i2cdetect -y 1
```

### Camera Not Detected

List available cameras:
```bash
ls /dev/video*
```

Edit `config/camera.yaml` to set correct camera index.

### GPIO Permission Issues

Add user to gpio group:
```bash
sudo usermod -a -G gpio $USER
```

### Object Detection Not Working

Ensure YOLO model is downloaded. The system will automatically download YOLOv5 on first run.

## License

MIT License

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.
