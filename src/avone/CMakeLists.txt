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
find_package(vision_msgs REQUIRED)        # ← add this


# Eigen is header-only; required by tf2_sensor_msgs
find_package(Eigen3 REQUIRED)

# —————————————————————————————————————————————
# avone hardware_interface plugin
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
  
  vision_msgs                # ← add here
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

# avone_can_interface executable
add_executable(avone_can_interface
  src/avone_can_interface_main.cpp
  src/avone_can_interface.cpp
)
target_include_directories(avone_can_interface PRIVATE include)
ament_target_dependencies(
  avone_can_interface
  rclcpp
  geometry_msgs
)
install(
  TARGETS avone_can_interface
  DESTINATION lib/${PROJECT_NAME}
)


# twiststamped executable
add_executable(twiststamped src/twiststamped.cpp)
ament_target_dependencies(
  twiststamped
  rclcpp
  geometry_msgs
)
install(TARGETS twiststamped DESTINATION lib/${PROJECT_NAME})



# —————————————————————————————————————————————



# —————————————————————————————————————————————
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

# Testing (if needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_avone_system_hardware test/test_avone_system_hardware.cpp)
  target_include_directories(test_avone_system_hardware PRIVATE include)
  ament_target_dependencies(
    test_avone_system_hardware
    hardware_interface
    pluginlib
    ros2_control_test_assets
    geometry_msgs
    nav_msgs
    
  )
endif()

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