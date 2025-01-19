cmake_minimum_required(VERSION 3.0.2)
project(block_controller)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros gazebo_msgs
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(block_controller src/block_controller.cpp)
target_link_libraries(block_controller ${catkin_LIBRARIES})