cmake_minimum_required(VERSION 3.0.2)
project(ft_detector)

# OpenCV 4 / ROS Noetic.  C++14 is enough and matches the assignment.
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  ft_detection
)
find_package(OpenCV REQUIRED)

# Generated from cfg/ArmorReconfigure.cfg.
generate_dynamic_reconfigure_options(
  cfg/ArmorReconfigure.cfg
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    cv_bridge
    dynamic_reconfigure
    geometry_msgs
    image_transport
    roscpp
    sensor_msgs
    tf2
    tf2_ros
    ft_detection
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_receiver src/image_receiver.cpp)

# The node includes the generated ArmorReconfigureConfig.h and the
# ft_detection/DetectionFrame.h message header.
add_dependencies(image_receiver
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(image_receiver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
