#!/usr/bin/env bash
set -e

echo "======================================"
echo "AV.ONE ROS 2 environment setup"
echo "======================================"

# ---- System basics ----
sudo apt update
sudo apt install -y \
  curl \
  git \
  gnupg \
  lsb-release \
  build-essential \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  libxml2-utils

# ---- ROS 2 Humble ----
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
fi

sudo apt update
sudo apt install -y ros-humble-desktop

# ---- rosdep ----
sudo rosdep init || true
rosdep update

# ---- Workspace deps ----
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# ---- Build ----
source /opt/ros/humble/setup.bash
colcon build

echo "======================================"
echo "AV.ONE setup complete"
echo "======================================"
