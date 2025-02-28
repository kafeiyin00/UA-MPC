cmake_minimum_required(VERSION 2.8.3)
project(odometry_generator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  roslib
  quadrotor_msgs
  tf
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS nav_msgs roscpp std_msgs
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/odometry_generator.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)