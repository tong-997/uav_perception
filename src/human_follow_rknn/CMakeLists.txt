cmake_minimum_required(VERSION 2.8.3)
project(human_follow_rknn)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  nav_msgs
  quadrotor_msgs
)

find_package(OpenCV REQUIRED)

## ----------------- 消息 -----------------
add_message_files(
  FILES
  TargetInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs geometry_msgs nav_msgs quadrotor_msgs
  INCLUDE_DIRS include
  LIBRARIES rknn_yolo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /usr/include       # rknn_api.h 所在路径，按你实际情况改
)

link_directories(
  /usr/lib           # librknnrt.so 所在路径，按你实际情况改
)

## ----------------- RKNN YOLO 封装库 -----------------
add_library(rknn_yolo
  src/rknn_yolo.cpp
)

target_link_libraries(rknn_yolo
  ${OpenCV_LIBS}
  rknnrt                # RKNN runtime 库名通常是 librknnrt.so
)

## ----------------- 跟踪节点 -----------------
add_executable(human_tracker_node
  src/human_tracker_node.cpp
)
add_dependencies(human_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(human_tracker_node
  rknn_yolo
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

## ----------------- 跟随控制节点 -----------------
add_executable(human_follow_px4ctrl_node
  src/human_follow_px4ctrl_node.cpp
)
add_dependencies(human_follow_px4ctrl_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(human_follow_px4ctrl_node
  ${catkin_LIBRARIES}
)