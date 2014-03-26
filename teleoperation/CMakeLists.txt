cmake_minimum_required(VERSION 2.8.3)
project(teleoperation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES turtlebot_teleop
  CATKIN_DEPENDS rospy geometry_msgs
  DEPENDS
)

###########
## Build ##
###########


#add_executable(teleop_omni_cartesian src/sensable_omni_cartesian.cpp)
#add_dependencies(teleop_omni_cartesian ${PROJECT_NAME}_gencfg)
#target_link_libraries(teleop_omni_cartesian ${catkin_LIBRARIES})

#add_executable(teleop_omni_cartesian src/sensable_omni_cartesian.cpp)
#target_link_libraries(teleop_omni_cartesian ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  scripts/cyber_glove_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
#install(TARGETS turtlebot_teleop_joy
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
