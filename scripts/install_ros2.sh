#!/bin/bash
set -e

echo "=== Setting Locale ==="
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== Enabling Ubuntu Universe Repository ==="
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

echo "=== Adding ROS2 GPG Key and APT Repository ==="
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== Installing ROS2 Jazzy Desktop ==="
sudo apt update
sudo apt install -y ros-jazzy-desktop

echo "=== Installing ROS2 Build Tools (colcon) ==="
sudo apt install -y python3-colcon-common-extensions python3-rosdep

echo "=== Registering Environment Variables to ~/.bashrc ==="
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "=== Initializing ROS2 Dependencies ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "ROS2 Jazzy and build tools installation complete. Open a new terminal or run 'source ~/.bashrc' to apply changes."
