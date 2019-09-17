cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs_visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  roscpp
  sensor_msgs
  vision_msgs
  visualization_msgs
  image_transport
  cv_bridge
  vision_info_server
)

catkin_package(
  INCLUDE_DIRS include
# LIBRARIES vision_msgs_visualization
  CATKIN_DEPENDS message_filters roscpp sensor_msgs vision_msgs visualization_msgs image_transport
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(detection_2d_visualizer_node src/detection_2d_visualizer_node.cpp src/detection_2d_visualizer.cpp)
target_link_libraries(detection_2d_visualizer_node ${catkin_LIBRARIES})
add_dependencies(detection_2d_visualizer_node ${catkin_EXPORTED_TARGETS})

add_executable(detection_3d_visualizer_node src/detection_3d_visualizer_node.cpp src/detection_3d_visualizer.cpp)
target_link_libraries(detection_3d_visualizer_node ${catkin_LIBRARIES})
add_dependencies(detection_3d_visualizer_node ${catkin_EXPORTED_TARGETS})

# Install library/executable
install(TARGETS detection_2d_visualizer_node detection_3d_visualizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)