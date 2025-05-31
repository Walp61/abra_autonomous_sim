cmake_minimum_required(VERSION 3.5)
project(abra_description)

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

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
  launch/abra.launch.py
  launch/spawn_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()