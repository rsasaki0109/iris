cmake_minimum_required(VERSION 3.10)
project(openvslam_bridge)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pipe -fopenmp -Ofast -lstdc++fs -mfpmath=both -mtune=native")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(message_filters REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(SYSTEM ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})

# Shared non-ROS bridge sources from the ros1 package
set(BRIDGE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../openvslam_bridge/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# OpenVSLAM (built as part of the ros1 openvslam_bridge submodule)
set(BUILD_EXAMPLES OFF)
set(USE_PANGOLIN_VIEWER OFF)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../openvslam_bridge/3rd/openvslam
                 ${CMAKE_CURRENT_BINARY_DIR}/openvslam)

include_directories(SYSTEM
  ${CMAKE_CURRENT_SOURCE_DIR}/../../openvslam_bridge/3rd/openvslam/src
)

add_executable(openvslam_bridge_node
  src/bridge_node.cpp
  ${BRIDGE_SRC}/bridge.cpp
)
target_link_libraries(openvslam_bridge_node openvslam ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(openvslam_bridge_node
  rclcpp image_transport cv_bridge sensor_msgs geometry_msgs tf2 tf2_ros pcl_conversions pcl_ros)

add_executable(openvslam_stereo_bridge_node
  src/stereo_bridge_node.cpp
  ${BRIDGE_SRC}/bridge.cpp
  ${BRIDGE_SRC}/stereo_bridge.cpp
)
target_link_libraries(openvslam_stereo_bridge_node openvslam ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(openvslam_stereo_bridge_node
  rclcpp image_transport cv_bridge sensor_msgs geometry_msgs tf2 tf2_ros
  pcl_conversions pcl_ros message_filters)

install(TARGETS
  openvslam_bridge_node
  openvslam_stereo_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
