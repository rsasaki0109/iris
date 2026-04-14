cmake_minimum_required(VERSION 3.10)
project(iris)

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
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io registration visualization kdtree)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(g2o REQUIRED)

set(G2O_LIBS
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_sim3
    g2o::solver_dense
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
    ${CXSPARSE_LIBRARIES}
    ${SUITESPARSE_LIBRARIES})

include_directories(SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})

# Shared algorithm sources (no ROS dependencies) from the ros1 package
set(IRIS_CORE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../iris/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)  # ros2-specific sources (must come first)
include_directories(${IRIS_CORE_SRC})

file(GLOB CORE_SOURCES
  "${IRIS_CORE_SRC}/core/*.cpp"
  "${IRIS_CORE_SRC}/map/*.cpp"
  "${IRIS_CORE_SRC}/optimize/*.cpp"
  "${IRIS_CORE_SRC}/pcl_/*.cpp"
  "${IRIS_CORE_SRC}/system/*.cpp"
)

add_executable(iris_node
  src/iris_node.cpp
  src/publish/publish.cpp
  ${CORE_SOURCES}
)

ament_target_dependencies(iris_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  pcl_ros
)

target_link_libraries(iris_node
  ${OpenCV_LIBS}
  ${G2O_LIBS}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS iris_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../config/
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING PATTERN "iris_*.yaml" PATTERN "*.rviz")

ament_package()
