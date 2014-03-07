cmake_minimum_required(VERSION 2.8.3)
project(use_devices)
find_package(catkin REQUIRED
  COMPONENTS message_generation sensor_msgs)

add_service_files(FILES
  FindCylinder.srv
  PointAtFrame.srv
  TurnKnob.srv
  UseButton.srv
  UseKnob.srv
  UseSwitch.srv
  )

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
 CATKIN_DEPENDS message_runtime
 )
