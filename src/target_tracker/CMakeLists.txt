cmake_minimum_required(VERSION 3.16)
project(target_tracker)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------
#  EKF 算法库（考生主要改这里）
# ---------------------------------------------------------------
add_library(kalman_ca SHARED src/kalman_filter.cpp)
target_include_directories(kalman_ca PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kalman_ca PUBLIC Eigen3::Eigen)

# 云台指令纯算法：世界系预测状态 -> yaw / yaw_vel，以及断流状态机。
add_library(gimbal_command SHARED src/gimbal_command.cpp)
target_include_directories(gimbal_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gimbal_command PUBLIC kalman_ca)

# ---------------------------------------------------------------
#  跟踪组件：EKF + 预测 + 云台指令
# ---------------------------------------------------------------
add_library(tracker_node_component SHARED src/tracker_node.cpp)
target_include_directories(tracker_node_component PRIVATE ${rclcpp_components_INCLUDE_DIRS})
target_link_libraries(tracker_node_component PRIVATE
  kalman_ca
  gimbal_command
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${interfaces_TARGETS})
rclcpp_components_register_node(tracker_node_component
  PLUGIN "target_tracker::TrackerNode"
  EXECUTABLE tracker_node)

# ---------------------------------------------------------------
#  安装
# ---------------------------------------------------------------
install(TARGETS kalman_ca gimbal_command
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS tracker_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------
#  单元测试：colcon test --packages-select target_tracker
# ---------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kalman_filter test/test_kalman_filter.cpp)
  target_link_libraries(test_kalman_filter kalman_ca)
  ament_add_gtest(test_gimbal_command test/test_gimbal_command.cpp)
  target_link_libraries(test_gimbal_command gimbal_command)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

ament_package()
