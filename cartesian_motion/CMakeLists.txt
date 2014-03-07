cmake_minimum_required(VERSION 2.8.3)
project(cartesian_motion)
find_package(catkin REQUIRED
  COMPONENTS message_generation geometry_msgs)

add_service_files(FILES
  MoveGripper.srv
  TurnGripper.srv
)
 
generate_messages(DEPENDENCIES geometry_msgs)

