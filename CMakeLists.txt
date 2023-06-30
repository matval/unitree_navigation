cmake_minimum_required(VERSION 3.0.2)
project(unitree_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  tf
  roscpp
  sensor_msgs
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS tf rospy roscpp message_runtime)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/arm64)

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

add_executable(imu_odom src/imu_odom.cpp)
target_link_libraries(imu_odom ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(imu_odom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
