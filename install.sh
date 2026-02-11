#!/bin/bash

# Clanker One-Command Installer
# Supported OS: Ubuntu 22.04 LTS and 64-bit Raspberry Pi OS with ROS2 Humble

set -e

echo "=========================================="
echo "      Clanker Hexapod Installation        "
echo "=========================================="

# Detect OS
if [ -f /etc/os-release ]; then
    . /etc/os-release
    OS_ID=$ID
    OS_VERSION=$VERSION_ID
    echo "Detected OS: $PRETTY_NAME"
else
    echo "Cannot detect OS. Exiting."
    exit 1
fi

# 1. Update System
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# 2. Install ROS2 Humble (if not installed)
if ! command -v ros2 &> /dev/null; then
    echo "ROS2 not found. Installing ROS2 Humble..."

    if [ "$OS_ID" = "ubuntu" ] && [ "$OS_VERSION" = "22.04" ]; then
        # Ubuntu 22.04 installation
        sudo apt install software-properties-common -y
        sudo add-apt-repository universe -y
        sudo apt update && sudo apt install curl -y
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install ros-humble-ros-base python3-colcon-common-extensions -y
    elif [ "$OS_ID" = "debian" ] || [ "$OS_ID" = "raspbian" ]; then
        # Raspberry Pi OS 64-bit (Debian-based) installation
        sudo apt update
        sudo apt install -y curl gnupg lsb-release
        sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
        sudo apt update
        sudo apt install -y \
            ros-humble-ros-base \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-pip
    else
        echo "Unsupported OS for automatic ROS2 installation. Please install ROS2 Humble manually."
        exit 1
    fi
else
    echo "ROS2 is already installed."
fi

# Source ROS2 environment
source /opt/ros/humble/setup.bash

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

# 4. Install Python dependencies via apt where available
echo "Installing Python packages via apt..."
sudo apt install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    libopenblas-dev \
    liblapack-dev \
    python3-scipy \
    python3-torch \
    python3-torchvision \
    libssl-dev \
    libffi-dev \
    python3-cryptography

# 5. Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# 6. Install Python Dependencies
echo "Installing Python packages..."
pip3 install --upgrade pip setuptools wheel

if [ -f "requirements.txt" ]; then
    echo "Installing from requirements.txt..."
    pip3 install -r requirements.txt
elif [ -f "hexapod_autonomous/requirements.txt" ]; then
    echo "Installing from hexapod_autonomous/requirements.txt..."
    pip3 install -r hexapod_autonomous/requirements.txt
fi

# 7. Enable I2C and Camera (for Raspberry Pi OS)
if command -v raspi-config &> /dev/null; then
    echo "Enabling I2C and Camera..."
    sudo raspi-config nonint do_i2c 0
    sudo raspi-config nonint do_camera 0
fi

# 8. Setup Workspace
echo "Building the workspace..."
colcon build --symlink-install --packages-select hexapod_autonomous

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
