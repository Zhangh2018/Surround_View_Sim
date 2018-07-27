cmake_minimum_required(VERSION 2.8.3)
project(surround_view_sim)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)
execute_process( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )

# Module Path
set(MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../modules)

# Stitch Path
set(STITCH_INCLUDE_DIRS ${MODULE_PATH}/sw_stitch/include)
set(STITCH_LIBS ${MODULE_PATH}/sw_stitch/lib/${ARCHITECTURE}/libsw_stitch.so
                ${MODULE_PATH}/sw_stitch/lib/${ARCHITECTURE}/libsw_calib.so
                ${MODULE_PATH}/sw_stitch/lib/${ARCHITECTURE}/libchessboard.so)

# 3rd party
set(3RD_PARTY_INCLUDE_DIRS /usr/local/mvp/include)
set(3RD_PARTY_LIB_DIRS /usr/local/mvp/lib)
set(3RD_PARTY_LIBS ${3RD_PARTY_LIB_DIRS}/libglog.so
                   ${3RD_PARTY_LIB_DIRS}/libgflags.so
                   ${3RD_PARTY_LIB_DIRS}/libceres.so)

find_package(catkin REQUIRED COMPONENTS
  can_msgs
  dbw_mkz_vision_sim
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  rviz
  sensor_msgs
  dbw_mkz_msgs
  std_msgs
  tf
  cv_bridge
)

catkin_package(
        CATKIN_DEPENDS  roscpp rospy std_msgs message_runtime
        DEPENDS #EIGEN3
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  include
  ${STITCH_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(stitch_ros src/stitch.cpp)
target_link_libraries(stitch_ros ${STITCH_LIBS} 
                                 ${OpenCV_LIBS}
                                 ${catkin_LIBRARIES}
                                 ${3RD_PARTY_LIBS}
)