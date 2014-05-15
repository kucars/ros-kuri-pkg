cmake_minimum_required(VERSION 2.8.3)
project(mitsubishi_arm_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  visualeyez_tracker
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES pose_broadcaster
#  CATKIN_DEPENDS roscpp rospy std_msgs
   CATKIN_DEPENDS geometry_msgs std_msgs visualeyez_tracker
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(mitsubishi_arm_teleop src/teleop.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mitsubishi_arm_teleop
   ${catkin_LIBRARIES}
 )
