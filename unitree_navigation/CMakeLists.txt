cmake_minimum_required(VERSION 3.0.2)
project(unitree_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	nav_msgs
	rospy
	tf2_ros
	roscpp
	sensor_msgs
	geometry_msgs
	unitree_legged_sdk
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS tf2_ros rospy roscpp)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  	set(ARCH amd64)
else()
  	set(ARCH arm64)
endif()

set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${unitree_legged_sdk_INCLUDE_DIRS}
)

add_executable(unitree_state_control src/unitree_state_control.cpp)
target_link_libraries(unitree_state_control ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(unitree_state_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
