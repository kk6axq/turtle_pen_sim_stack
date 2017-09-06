cmake_minimum_required(VERSION 2.8.3)
project(turtle_pen_sim_msgs)


find_package(catkin REQUIRED message_generation)


add_service_files(
   FILES
   TwoInts.srv
   Teleport.srv
)

add_message_files(
   FILES
   PenState.msg    
)
## Generate added messages and services with any dependencies listed here
generate_messages(
)



catkin_package(
  CATKIN_DEPENDS message_runtime
)



