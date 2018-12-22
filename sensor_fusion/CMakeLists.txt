cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  roscpp
  rospy
  sensor_msgs
  pcl_ros
  jsk_recognition_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
	set (CMAKE_C_FLAG "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velodyne_camera_calibration
#  CATKIN_DEPENDS cv_bridge image_transport message_filters roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  fusion
  ${catkin_INCLUDE_DIRS}
)

add_executable(matching_points_image src/matching_points_image.cpp)
target_link_libraries(matching_points_image
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_+IBRARIES}
)

add_executable(points_matching_image src/points_matching_image.cpp)
target_link_libraries(points_matching_image
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_+IBRARIES}
)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
target_link_libraries(camera_info_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_+IBRARIES}

)

# add_executable(measure_range src/measure_range.cpp)
# target_link_libraries(camera_info_publisher
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   ${OpenCV_+IBRARIES}

# )
