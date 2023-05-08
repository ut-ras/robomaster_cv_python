cmake_minimum_required(VERSION 3.5)
project(sentry_mecanum)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

########################
##### DEPENDENCIES #####
########################
# Base
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ROS
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Gazebo
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)

# Include
include_directories(include)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# Gazebo Swerve Odometry Plugin
add_library(mecanum_plugin SHARED src/mecanum_plugin.cpp)
ament_target_dependencies(mecanum_plugin
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  gazebo
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  gazebo_plugins
)
target_link_libraries(mecanum_plugin ${GAZEBO_LIBRARIES})
target_include_directories(mecanum_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_export_libraries(mecanum_plugin)

# ################
# ### Install ####
# ################
install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME})

ament_package()
