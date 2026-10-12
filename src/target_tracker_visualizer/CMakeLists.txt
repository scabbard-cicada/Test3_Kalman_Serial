cmake_minimum_required(VERSION 3.16)
project(target_tracker_visualizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 可视化组件：发布 Path / Marker / MarkerArray 和 TF，交给 Foxglove/RViz 的
# 3D 面板显示。只读算法与串口话题，不改动任何接口。
add_library(visualizer_node_component SHARED src/visualizer_node.cpp)
target_include_directories(visualizer_node_component PRIVATE ${rclcpp_components_INCLUDE_DIRS})
target_link_libraries(visualizer_node_component PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)
rclcpp_components_register_node(visualizer_node_component
  PLUGIN "target_tracker_visualizer::VisualizerNode"
  EXECUTABLE visualizer_node)

install(TARGETS visualizer_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
