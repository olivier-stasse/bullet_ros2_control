cmake_minimum_required(VERSION 3.8)
SET(PROJECT_NAME bullet_ros2_control)
project(${PROJECT_NAME})

SET(CMAKE_VERBOSE_MAKEFILE TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(Bullet REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
include_directories(${BULLET_INCLUDE_DIRS})
link_directories(
  ${BULLET_LIBRARY_DIRS}
)

# Library
add_library(bullet_hardware_plugin SHARED
  src/bullet_system.cpp
)
ament_target_dependencies(bullet_hardware_plugin
  hardware_interface
  rclcpp
  urdf
)

target_link_libraries(bullet_hardware_plugin BulletRobotics)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
ament_package()
