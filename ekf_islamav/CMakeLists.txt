cmake_minimum_required(VERSION 2.8.3)
project(ekf_islamav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  tf
)

#add_message_files(FILES mensaje_posicion.msg) 
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   filter_state.msg
   Navdata.msg
   mensaje_kalman.msg
   mensaje_kalman_array.msg
   Ardrone3PilotingStateAltitudeChanged.msg
   Ardrone3PilotingStateAttitudeChanged.msg
   Ardrone3PilotingStateSpeedChanged.msg
   vector31.msg
   vector21.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_message_files(FILES filter_state.msg)
add_message_files(FILES Navdata.msg)
add_message_files(FILES mensaje_kalman.msg)
add_message_files(FILES mensaje_kalman_array.msg)
add_message_files(FILES Ardrone3PilotingStateAltitudeChanged.msg)
add_message_files(FILES Ardrone3PilotingStateAttitudeChanged.msg)
add_message_files(FILES Ardrone3PilotingStateSpeedChanged.msg)
add_message_files(FILES vector31.msg)
add_message_files(FILES vector21.msg)



## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES utiles
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#Añado los nodos que quiera

add_executable(ekf_islamav src/ekf_islamav.cpp)
target_link_libraries(ekf_islamav ${catkin_LIBRARIES})
add_dependencies(ekf_islamav ekf_islamav_generate_messages_cpp)
