cmake_minimum_required(VERSION 3.0.2)
project(ft_tracker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  ft_detection
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)

add_message_files(FILES Track.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    ft_detection
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(tracker_node src/tracker_node.cpp)
add_dependencies(tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
