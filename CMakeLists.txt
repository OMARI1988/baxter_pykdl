cmake_minimum_required(VERSION 2.8.3)
project(baxter_pykdl)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
  baxter_core_msgs
  baxter_interface
  message_generation
)

catkin_python_setup()


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   joy_stick_commands.msg
#   Message1.msg
#   Message2.msg
 )


 ## Generate added messages and services with any dependencies listed here
  generate_messages(
    DEPENDENCIES
    std_msgs
    #qsr
  )

  catkin_package(
    CATKIN_DEPENDS
    rospy
    geometry_msgs
    sensor_msgs
    urdfdom
    tf
    baxter_interface
    baxter_core_msgs
    DEPENDS
    python_orocos_kdl
    urdfdom
  )
  
install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
