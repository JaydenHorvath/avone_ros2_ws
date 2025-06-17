cmake_minimum_required(VERSION 3.8)
project(ground_removal_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(sensor_msgs  REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros      REQUIRED)
find_package(PCL COMPONENTS common io filters segmentation REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(ground_removal_node src/ground_removal_node.cpp)

ament_target_dependencies(ground_removal_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

# Link against PCL libraries
target_link_libraries(ground_removal_node
  ${PCL_LIBRARIES}
)

install(
  TARGETS ground_removal_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()