#!/usr/bin/env bash
set -e

echo "======================================"
echo "AV.ONE ROS 2 Humble Full Setup"
echo "======================================"

# -----------------------------
# Sanity checks
# -----------------------------
if [[ "$(lsb_release -cs)" != "jammy" ]]; then
  echo "ERROR: Ubuntu 22.04 (jammy) required"
  exit 1
fi

if [[ $EUID -eq 0 ]]; then
  echo "ERROR: Do not run this script as root"
  exit 1
fi

# -----------------------------
# Locale
# -----------------------------
echo "[INFO] Setting up locale"

sudo apt update
sudo apt install -y locales curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# -----------------------------
# ROS 2 repository
# -----------------------------
echo "[INFO] Adding ROS 2 repository"

sudo apt install -y software-properties-common

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# -----------------------------
# Install ROS 2 Humble
# -----------------------------
echo "[INFO] Installing ROS 2 Humble Desktop"

sudo apt install -y ros-humble-desktop

# -----------------------------
# Source ROS
# -----------------------------
source /opt/ros/humble/setup.bash
export ROS_DISTRO=humble

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc || \
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# -----------------------------
# Gazebo + ros2_control bridge
# -----------------------------
echo "[INFO] Installing Gazebo and ros2_control bridge"

sudo apt install -y \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-gz-ros2-control

# -----------------------------
# Core control, navigation, tools
# -----------------------------
echo "[INFO] Installing control, navigation, and tools"

sudo apt install -y \
  ros-humble-controller-manager \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-localization \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-topic-tools

# -----------------------------
# ROSIDL typesupport (CRITICAL)
# -----------------------------
echo "[INFO] Installing ROSIDL typesupport"

sudo apt install -y \
  ros-${ROS_DISTRO}-rosidl-typesupport-c \
  ros-${ROS_DISTRO}-rosidl-typesupport-cpp

# -----------------------------
# Build + Python tooling
# -----------------------------
echo "[INFO] Installing build and Python tooling"

sudo apt install -y \
  build-essential \
  cmake \
  git \
  cython3 \
  python3-dev \
  python3-pip \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-argcomplete \
  python3-ament-package

# Prevent pip from shadowing ROS Python
grep -qxF "export PYTHONNOUSERSITE=1" ~/.bashrc || \
  echo "export PYTHONNOUSERSITE=1" >> ~/.bashrc
export PYTHONNOUSERSITE=1

# -----------------------------
# rosdep
# -----------------------------
echo "[INFO] Initializing rosdep"

sudo apt install -y python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# -----------------------------
# Workspace check
# -----------------------------
WORKSPACE="$HOME/ros2_ws"

if [ ! -d "$WORKSPACE/src" ]; then
  echo "ERROR: Workspace not found at $WORKSPACE/src"
  exit 1
fi

# -----------------------------
# System deps not handled by rosdep
# -----------------------------
echo "[INFO] Installing additional system dependencies"

sudo apt install -y python3-pcl || \
  echo "[WARN] python3-pcl not available, skipping"

# -----------------------------
# Resolve ROS deps
# -----------------------------
echo "[INFO] Installing workspace dependencies via rosdep"

rosdep install \
  --from-paths "$WORKSPACE/src" \
  --ignore-src \
  -r \
  -y \
  --rosdistro humble

# -----------------------------
# Python ML + transforms
# -----------------------------
echo "[INFO] Installing Python ML dependencies"

pip3 uninstall -y numpy || true
pip3 install --user "numpy<2.0"

pip3 install --user --upgrade \
  ultralytics \
  tf-transformations

# -----------------------------
# Build workspace
# -----------------------------
echo "[INFO] Building workspace"

cd "$WORKSPACE"
rm -rf build install log

colcon build --symlink-install

# -----------------------------
# Shell setup
# -----------------------------
grep -qxF "source $WORKSPACE/install/setup.bash" ~/.bashrc || \
  echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# -----------------------------
# Final verification
# -----------------------------
source "$WORKSPACE/install/setup.bash"

ros2 --version
python3 -c "import ament_package; print('ament_package OK')"

echo "======================================"
echo "AV.ONE ROS 2 setup complete"
echo "======================================"
