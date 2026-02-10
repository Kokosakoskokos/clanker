# Clanker: Autonomous Hexapod Robot

Clanker is a ROS2-based autonomous hexapod robot designed for Raspberry Pi. It features face recognition, person following, voice control, and AI-powered decision making.

## Quick Start (One-Command Install)

To install Clanker on your Raspberry Pi (running Ubuntu 22.04 with ROS2 Humble), run the following command:

```bash
curl -sSL https://raw.githubusercontent.com/Kokosakoskokos/clanker/master/install.sh | bash
```

## Features

- **Autonomous Navigation**: Obstacle avoidance using ultrasonic sensors.
- **Computer Vision**: Face recognition and object detection (YOLOv5).
- **Person Following**: Automatically follows recognized individuals.
- **Voice Control**: Natural language interaction in Czech.
- **AI Brain**: Integrated with LLMs via OpenRouter for intelligent responses.
- **Modular Design**: Built on ROS2 for high reliability and extensibility.

## Hardware Requirements

- Raspberry Pi 4 (8GB recommended)
- 18x MG995 Servo Motors
- 2x PCA9685 I2C Servo Controllers
- USB Camera
- 4x HC-SR04 Ultrasonic Sensors
- 7.4V LiPo Battery

## Software Architecture

Clanker is built using ROS2 Humble and Python. It consists of several modular nodes:

- **hexapod_bringup**: Main entry point for the system.
- **hexapod_control**: Gait generation and servo control.
- **hexapod_vision**: Camera processing, face recognition, and object detection.
- **hexapod_perception**: Ultrasonic sensor data processing.
- **hexapod_navigation**: Path planning and person following.
- **hexapod_ai**: Voice recognition, TTS, and LLM brain.

## Configuration

Configuration files are located in the `config/` directory:
- `servos.yaml`: Calibrate your servos and pin assignments.
- `ai_config.yaml`: Set your OpenRouter API key and language preferences.
- `navigation.yaml`: Adjust movement and following parameters.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
