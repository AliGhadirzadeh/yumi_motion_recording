cmake_minimum_required(VERSION 2.8.3)
project(yumi_motion_recording)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  kdl_wrapper
  roscpp
)
find_package(orocos_kdl REQUIRED)

catkin_package(
  DEPENDS orocos_kdl system_lib
  CATKIN_DEPENDS  kdl_parser kdl_wrapper roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})
find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})

add_executable(record_motion_node src/record_motion.cpp)
target_link_libraries(record_motion_node  ${catkin_LIBRARIES} )
