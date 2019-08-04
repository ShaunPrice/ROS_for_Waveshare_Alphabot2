cmake_minimum_required(VERSION 3.1)
project(waveshare_alphabot2)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  compressed_image_transport
  roscpp
  std_msgs
  camera_info_manager
  rospy
  sensor_msgs
  dynamic_reconfigure
)
find_package(Boost REQUIRED)
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
  FILES
  IR.msg
  Line_Follow.msg
  Line_Follow_Stamped.msg
  Obstacle_Stamped.msg
  Pan_Tilt.msg
  Pan_Tilt_Stamped.msg
  RGB_LED.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(

)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    message_runtime
    rospy
    std_msgs
    sensor_msgs
    message_runtime
  DEPENDS Boost
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/ir_control
  nodes/joystick
  nodes/motion
  nodes/pan_tilt
  nodes/rgb_leds
  nodes/sensors
  nodes/sound
  scripts/test.py
  
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  nodes
  scripts
  rviz
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)

endif()

