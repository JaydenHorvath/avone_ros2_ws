cmake_minimum_required(VERSION 3.5)
project(avone)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ament and ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Find dependencies for the overlay node
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Eigen is header-only; required by tf2_sensor_msgs
find_package(Eigen3 REQUIRED)

# ------------------------------------------------------------
# avone hardware_interface plugin (shared library)
add_library(
  avone
  SHARED
  src/avone_system_hardware.cpp
  src/avone_can_interface.cpp
)

target_include_directories(
  avone
  PUBLIC
    include
)

ament_target_dependencies(
  avone
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  vision_msgs
)

target_compile_definitions(avone PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface avone.xml
)

install(
  TARGETS avone
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# ------------------------------------------------------------
# Install include directory
install(
  DIRECTORY include/
  DESTINATION include
)

# Install other resource folders
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(avone)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
)

ament_package()
