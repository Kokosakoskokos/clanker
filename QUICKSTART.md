# Quick Start Guide

## 1. Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-pip python3-rosdep
sudo apt install -y i2c-tools python3-smbus
sudo apt install -y python3-opencv python3-numpy
sudo apt install -y portaudio19-dev python3-pyaudio
sudo apt install -y libespeak1 python3-espeak
```

## 2. Clone and Setup

```bash
cd ~
git clone <your-repo-url>
cd hexapod_autonomous
pip3 install -r requirements.txt
```

## 3. Enable Hardware

```bash
sudo raspi-config
```

Enable: I2C, Camera, Serial

## 4. Build

```bash
cd ~/hexapod_autonomous
colcon build --symlink-install
source install/setup.bash
```

## 5. Test Individual Components

Test servo control:
```bash
ros2 run hexapod_autonomous hexapod_control
```

Test camera:
```bash
ros2 run hexapod_autonomous hexapod_vision
```

Test voice (requires microphone):
```bash
export OPENAI_API_KEY="your_key"
ros2 run hexapod_autonomous hexapod_voice
```

## 6. Launch Full System

```bash
ros2 launch hexapod_autonomous hexapod_full.launch.py
```

## Voice Commands

Start with "robot" keyword:
- "robot go" - Start autonomous mode
- "robot stop" - Stop
- "robot forward" - Move forward
- "robot left" - Turn left
- "robot what do you see" - List objects

## Common Issues

**No servo movement:**
- Check I2C: `i2cdetect -y 1`
- Verify PCA9685 addresses in config/servos.yaml

**Camera not working:**
- Check camera: `ls /dev/video*`
- Update camera index in config/camera.yaml

**Voice not recognized:**
- Test microphone: `arecord -f cd -d 5 test.wav`
- Install: `sudo apt install pulseaudio`

## Next Steps

1. Calibrate servos using config/servos.yaml
2. Adjust gait parameters in config/gait_params.yaml
3. Configure sensor settings in config/navigation.yaml
4. Set up AI key for intelligent voice commands
