cmake_minimum_required(VERSION 2.8.3)
project(device_interface)
find_package(catkin REQUIRED
  COMPONENTS message_generation visualization_msgs sensor_msgs)

add_service_files(FILES
  LevelTF.srv
  SendMarker.srv
  SendPointCloud.srv
  SendTF.srv
  )

generate_messages(DEPENDENCIES visualization_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
  )
  

