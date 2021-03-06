cmake_minimum_required(VERSION 2.8.3)
project(navigation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  visualization_msgs
  tf
  phantom_omni
  dynamic_reconfigure
  message_generation
  laser_geometry
  gazebo_msgs
ardrone_autonomy
 )

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PIDController.cfg
  cfg/MasterController.cfg
  cfg/SlaveController.cfg
  cfg/ForceField.cfg
  cfg/potential_field.cfg

  #...
)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   TwistArray.msg
   ContourData.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder  nav_msgs
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs  sensor_msgs nav_msgs visualization_msgs ardrone_autonomy
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
#  INCLUDE_DIRS include
#  LIBRARIES rosaria_nav
  CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs gazebo_msgs ardrone_autonomy
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
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(controller
   src/Controller.cpp
   src/MasterController.cpp
   src/SlaveController.cpp
)

## Declare a cpp executable
# add_executable(rosaria_nav_node src/rosaria_nav_node.cpp)
#add_executable(transform src/transform.cpp)

#add_executable(PID_position_controller src/PID_PositionControl.cpp)
#add_dependencies(PID_position_controller uav_commander_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#target_link_libraries(PID_position_controller  ${catkin_LIBRARIES} -lpq )


add_executable(force_field src/force_field.cpp)
add_dependencies(force_field ${PROJECT_NAME}_gencfg)
target_link_libraries(force_field
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )

add_executable(potential_field src/potential_field.cpp)
add_dependencies(potential_field ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(potential_field
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)


add_executable(laser2pointcloud src/laser2pointcloud.cpp)
add_dependencies(laser2pointcloud ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(laser2pointcloud
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

## take off and landing 
add_executable(takeoff src/take_off.cpp)
add_dependencies(takeoff ${PROJECT_NAME}_gencfg)
target_link_libraries(takeoff
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)
add_executable(land src/land.cpp)
add_dependencies(land ${PROJECT_NAME}_gencfg)
target_link_libraries(land
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

#add_executable(laser2PC2 src/laser2PC2.cpp)
#add_dependencies(laser2PC2 ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
#target_link_libraries(laser2PC2
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#)
add_executable(controller_ros src/ControllerRos.cpp)
add_dependencies(controller_ros ${PROJECT_NAME}_gencfg)
target_link_libraries(controller_ros
   controller
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

add_executable(ModelState src/ModelState.cpp)
add_dependencies(ModelState ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(ModelState
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)


#add_executable(uav_commander src/uav_commander.cpp)
#target_link_libraries(uav_commander
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#)
