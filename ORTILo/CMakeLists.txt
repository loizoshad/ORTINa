cmake_minimum_required(VERSION 3.0.2)
project(ORTILo)


find_package(catkin REQUIRED COMPONENTS
  #darknet_ros
  #darknet_ros_msgs
  dji_m100_controllers_gazebo
  dji_m100_description
  dji_m100_gazebo
  geometry_msgs
  roscpp
  rospy
  std_msgs
  mav_msgs
  move_base
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
  uwb_driver
  path_smoothing_ros
)

add_message_files(
  FILES
  yolov3_bounding_box.msg
  yolov3_bounding_box_vector.msg
  ControlVector.msg
  StateVector.msg
)

add_action_files(
FILES
Pose.action
)

generate_messages(
DEPENDENCIES
actionlib_msgs
std_msgs
nav_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ORTILo lq_servo
CATKIN_DEPENDS path_smoothing_ros uwb_driver dji_m100_controllers_gazebo dji_m100_description dji_m100_gazebo geometry_msgs roscpp rospy std_msgs mav_msgs move_base nav_msgs actionlib actionlib_msgs message_generation message_runtime
#CATKIN_DEPENDS darknet_ros darknet_ros_msgs dji_m100_controllers_gazebo dji_m100_description dji_m100_gazebo geometry_msgs roscpp rospy std_msgs mav_msgs move_base nav_msgs actionlib actionlib_msgs message_generation
#DEPENDS system_lib
DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lq_servo
  src/library/lq_servo.cpp
)

target_link_libraries(lq_servo ${catkin_LIBRARIES})
add_dependencies(lq_servo ${catkin_EXPORTED_TARGETS})


#add_executable(motor_pub_1 src/motor_pub_1)
add_executable(pose_action_server src/pose_action_server)
add_executable(pose_action_client src/pose_action_client)
add_executable(path_smoothing_node src/path_smoothing_node)

#add_executable(newton_raphson src/newton_raphson)
#target_link_libraries(newton_raphson ${catkin_LIBRARIES})

#target_link_libraries(motor_pub_1 ${catkin_LIBRARIES})
target_link_libraries(pose_action_server ${catkin_LIBRARIES})
target_link_libraries(pose_action_client ${catkin_LIBRARIES})
target_link_libraries(path_smoothing_node ${catkin_LIBRARIES})

add_executable(lq_servo_node src/nodes/lq_servo_node.cpp)
target_link_libraries(lq_servo_node ${catkin_LIBRARIES})
add_dependencies(lq_servo_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(lq_servo_node
lq_servo ${catkin_LIBRARIES})



#############
## Install ##
#############

install(TARGETS lq_servo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lq_servo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ORTILo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
