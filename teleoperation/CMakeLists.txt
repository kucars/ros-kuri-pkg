cmake_minimum_required(VERSION 2.8.3)
project(teleoperation)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs phantom_omni dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/HapticTeleop.cfg
  #...
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
#  LIBRARIES turtlebot_teleop
  CATKIN_DEPENDS roscpp geometry_msgs phantom_omni
  DEPENDS
)



###########
## Build ##
###########

add_executable(teleop_joy src/joy.cpp)
add_dependencies(teleop_joy ${PROJECT_NAME}_gencfg)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})

add_executable(teleop_keyboard src/keyboard.cpp)
add_dependencies(teleop_keyboard ${PROJECT_NAME}_gencfg)
target_link_libraries(teleop_keyboard ${catkin_LIBRARIES})

add_executable(teleop_omni src/sensable_omni.cpp)
add_dependencies(teleop_omni ${PROJECT_NAME}_gencfg)
target_link_libraries(teleop_omni ${catkin_LIBRARIES})

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
