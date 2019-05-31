cmake_minimum_required(VERSION 2.8.3)
project(spo_recognition)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  rospy
  bica
  robocuphomeeducation_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spo_recognition
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(spo_recognition_executor_node
  src/spo_recognition_executor_node.cpp
  src/spo_recognition_executor.cpp
  src/spo_recognition_HFSM.cpp
)

add_dependencies(spo_recognition_executor_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(spo_recognition_executor_node
 ${catkin_LIBRARIES}
)
