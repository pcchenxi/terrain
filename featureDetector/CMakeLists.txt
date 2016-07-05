cmake_minimum_required(VERSION 2.8.3)
project(featureDetector)

set (OpenCV_DIR ~/opencv/opencv3.0.0/share/OpenCV)
set (PCL_DIR ~/opencv/pcl/share/pcl-1.8)

find_package(OpenCV 3.0.0 REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES featureDetector
#  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ~/opencv/pcl/include/pcl-1.8
  ~/opencv/opencv3.0.0/include
  ~/opencv/opencv3.0.0/build/opencv_contrib/modules/ximgproc/include
  ~/opencv/opencv3.0.0/build/opencv_contrib/modules/xfeatures2d/include
  src
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ~/opencv/pcl/build/lib
  ~/opencv/opencv3.0.0/build/lib
  ${PCL_LIBRARY_DIRS}
)

## Declare a cpp executable
 add_executable(featureDetector_node src/featureDetector.cpp src/slic.cpp)

add_definitions(${PCL_DEFINITIONS})

## Specify libraries to link a library or executable target against
 target_link_libraries(featureDetector_node
   opencv_ximgproc
   opencv_xfeatures2d
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

