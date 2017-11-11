cmake_minimum_required(VERSION 2.8.3)
project(dxl_cntl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  joy_cntl
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dxl_motor
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
)

add_executable(${PROJECT_NAME} src/dxl_cntl.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} /usr/local/lib/libdxl_x64_cpp.so)

#############
## Install ##
#############

#############
## Testing ##
#############
