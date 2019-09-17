cmake_minimum_required(VERSION 2.8.3)
project(packml_stats_loader)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  packml_msgs
  packml_ros
  packml_sm
  roscpp
  rosbag
)

catkin_package(
  CATKIN_DEPENDS
    packml_msgs
    packml_ros
    packml_sm
    roscpp
    rosbag
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/packml_stats_loader.cpp
)
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME} src/packml_stats_loader_node.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
