cmake_minimum_required(VERSION 2.8.3)
project(mitsubishi_barrett_hardware_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  barrett_hand_hardware_interface
  mitsubishi_arm_hardware_interface
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mitsubishi_barrett_hardware_interface
#  CATKIN_DEPENDS roscpp rospy
  DEPENDS lapack blas gfortran
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(mitsubishi_barrett_hardware_interface src/MitsubishiBarrettHardwareInterface.cpp)
add_executable(mitsubishi_barrett_hardware_interface_node src/Robot.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(mitsubishi_barrett_hardware_interface_node mitsubishi_barrett_hardware_interface ${catkin_LIBRARIES}  lapack blas gfortran)


