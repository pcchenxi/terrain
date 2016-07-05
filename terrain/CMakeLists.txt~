cmake_minimum_required(VERSION 2.8.3)
project(terrain)

find_package(PCL 1.7 REQUIRED COMPONENTS)
find_package(OpenCV 2.4.8 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  laser_geometry
)

add_definitions("-g")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES terrain
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

set(PROJECT_INCLUDE
src/functions/continuity_filter.h
src/functions/cross_section_filter.h
src/functions/histogram_filter.h
src/functions/normal_filter.h
src/functions/terrain_function_sturcture.h
src/functions/xi_functions.h
)

add_executable(pointshape_based_processor src/pointshape_based_processor.cpp ${PROJECT_INCLUDE})

target_link_libraries(pointshape_based_processor
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)





