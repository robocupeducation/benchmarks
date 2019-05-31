cmake_minimum_required(VERSION 2.8.3)
project(restaurant)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  rospy
  bica
  robocuphomeeducation_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES restaurant
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

add_executable(restaurant_executor_node
  src/restaurant_executor_node.cpp
  src/restaurant_executor.cpp
  src/restaurant_HFSM.cpp
)

add_dependencies(restaurant_executor_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(restaurant_executor_node
 ${catkin_LIBRARIES}
)
