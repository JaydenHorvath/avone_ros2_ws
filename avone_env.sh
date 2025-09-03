export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# ~/ros2_ws/avone_env.sh

# Match AV.ONE's settings
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
unset RMW_IMPLEMENTATION    # We'll start with Fast-DDS like AV.ONE

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source your laptop's workspace if it has custom msgs needed for AV.ONE
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

