#!/bin/bash

# Clanker One-Command Installer
# Supported OS: Ubuntu 22.04 LTS with ROS2 Humble

set -e

echo "=========================================="
echo "      Clanker Hexapod Installation        "
echo "=========================================="

# 1. Update System
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# 2. Install ROS2 Humble (if not installed)
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 not found. Installing ROS2 Humble..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install ros-humble-ros-base python3-colcon-common-extensions -y
fi

# 3. Install System Dependencies
echo "Installing system dependencies..."
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-opencv \
    python3-numpy \
    python3-yaml \
    i2c-tools \
    python3-smbus2 \
    portaudio19-dev \
    python3-pyaudio \
    libespeak1 \
    espeak \
    flac \
    alsa-utils \
    python3-rpi.gpio \
    python3-serial

# 4. Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 5. Install Python Dependencies
echo "Installing Python packages..."
pip3 install --upgrade pip
if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt
elif [ -f "hexapod_autonomous/requirements.txt" ]; then
    pip3 install -r hexapod_autonomous/requirements.txt
fi

# 6. Enable I2C and Camera (for Raspberry Pi OS / Ubuntu with raspi-config)
if command -v raspi-config &> /dev/null; then
    echo "Enabling I2C and Camera..."
    sudo raspi-config nonint do_i2c 0
    sudo raspi-config nonint do_camera 0
fi

# 7. Setup Workspace
echo "Building the workspace..."
colcon build --symlink-install

echo ""
echo "=========================================="
echo "         Installation Complete!           "
echo "=========================================="
echo ""
echo "Please add the following to your ~/.bashrc:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ~/clanker/install/setup.bash"
echo ""
echo "Don't forget to set your OpenRouter API Key:"
echo "  export OPENROUTER_API_KEY='your_key_here'"
echo ""
echo "To start Clanker:"
echo "  ros2 launch hexapod_autonomous hexapod_full.launch.py"
echo ""
