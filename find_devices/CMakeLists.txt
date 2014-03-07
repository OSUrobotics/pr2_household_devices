cmake_minimum_required(VERSION 2.8.3)
project(find_devices)
find_package(catkin REQUIRED
  COMPONENTS message_generation roscpp tf sensor_msgs pcl_ros   )

# PCL Stuff
find_package(PCL REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_service_files(FILES
  FilterCloudByAxes.srv
  FindButton.srv
  FindDevice.srv
  FindKnob.srv
  FindSwitch.srv
  FitPlane.srv
  GetKnobParams.srv
  TransformPointCloud2.srv)


generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
 CATKIN_DEPENDS message_runtime
 )


# For devices in general
add_executable(plane_finder src/plane_finder.cpp)
target_link_libraries (plane_finder ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(plane_finder ${PROJECT_NAME}_generate_messages_cpp)

add_executable(filter_cloud_by_axes src/filter_cloud_by_axes.cpp)
target_link_libraries (filter_cloud_by_axes ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(filter_cloud_by_axes ${PROJECT_NAME}_generate_messages_cpp)


# For knobs
add_executable(transform_PointCloud2 src/transform_PointCloud2.cpp)
target_link_libraries (transform_PointCloud2 ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(transform_PointCloud2 ${PROJECT_NAME}_generate_messages_cpp)

add_executable(get_knob_params src/get_knob_params.cpp)
target_link_libraries (get_knob_params ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(get_knob_params ${PROJECT_NAME}_generate_messages_cpp)

