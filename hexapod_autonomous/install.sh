# Installation Script for Raspberry Pi

#!/bin/bash

echo "=========================================="
echo "Hexapod Autonomous Robot Installation"
echo "=========================================="

# Update system
echo "[1/6] Updating system..."
sudo apt update && sudo apt upgrade -y

# Install ROS2 dependencies
echo "[2/6] Installing ROS2 dependencies..."
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    i2c-tools \
    python3-smbus2

# Install audio dependencies
echo "[3/6] Installing audio dependencies..."
sudo apt install -y \
    portaudio19-dev \
    python3-pyaudio \
    libespeak1 \
    espeak \
    flac \
    alsa-utils

# Install hardware dependencies
echo "[4/6] Installing hardware dependencies..."
sudo apt install -y \
    python3-rpi.gpio \
    python3-serial

# Install Python packages
echo "[5/6] Installing Python packages..."
pip3 install --upgrade pip
pip3 install -r requirements.txt

# Enable I2C and Camera
echo "[6/6] Enabling I2C and Camera..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Please reboot your Raspberry Pi:"
echo "  sudo reboot"
echo ""
echo "After reboot, run:"
echo "  cd ~/hexapod_autonomous"
echo "  colcon build --symlink-install"
echo "  source install/setup.bash"
echo ""
echo "To start the robot:"
echo "  ros2 launch hexapod_autonomous hexapod_full.launch.py"
echo ""
