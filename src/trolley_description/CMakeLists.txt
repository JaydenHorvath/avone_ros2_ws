cmake_minimum_required(VERSION 3.8)
project(trolley_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------- Dependencies -------------------
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)

# ------------------- Install files -------------------
# Copy URDF, launch, and RViz folders into install/share directory
install(DIRECTORY
  description
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ------------------- Optional linting -------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ------------------- Finalize package -------------------
ament_package()
