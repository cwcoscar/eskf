cmake_minimum_required(VERSION 3.0.2)
project(eskf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  sensor_msgs
  geometry_msgs
  ublox_msgs
  uwb_ins_eskf_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(DIRECTORY msg)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
 CATKIN_DEPENDS 
 roscpp 
 rospy 
 std_msgs 
 message_runtime
 sensor_msgs
 uwb_ins_eskf_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_eskf src/main.cpp src/eskf.cpp)

target_link_libraries(main_eskf ${catkin_LIBRARIES})

add_dependencies(main_eskf
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS
main_eskf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )