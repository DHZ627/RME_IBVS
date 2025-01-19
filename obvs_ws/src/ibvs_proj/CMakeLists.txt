cmake_minimum_required(VERSION 3.0.2)
project(ibvs_proj)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  rospy
  roscpp
  sensor_msgs
  std_msgs
  mavros_msgs
  message_generation
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN3 REQUIRED eigen3)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  img_moment.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport message_filters mavros_msgs message_runtime
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${EIGEN_DEFINITIONS})

add_executable(image_processor_node src/image_processor_node.cpp)
target_link_libraries(image_processor_node ${catkin_LIBRARIES})

add_executable(ibvs_node src/IBVS_node.cpp)
target_link_libraries(ibvs_node ${catkin_LIBRARIES})

add_executable(observe_node src/observer.cpp)
target_link_libraries(observe_node ${catkin_LIBRARIES})