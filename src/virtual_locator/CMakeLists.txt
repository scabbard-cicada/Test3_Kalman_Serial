cmake_minimum_required(VERSION 3.16)
project(virtual_locator)

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
find_package(nlohmann_json REQUIRED)

# 观测回放组件：读 JSON 发极坐标观测。既能独立跑，也能装进 tracker_launcher 的容器。
add_library(virtual_locator_component SHARED src/measurement_publisher.cpp)
target_include_directories(virtual_locator_component PRIVATE ${rclcpp_components_INCLUDE_DIRS})
target_link_libraries(virtual_locator_component PRIVATE
  rclcpp::rclcpp
  ${interfaces_TARGETS}
  nlohmann_json::nlohmann_json)
rclcpp_components_register_node(virtual_locator_component
  PLUGIN "virtual_locator::MeasurementPublisher"
  EXECUTABLE virtual_locator)

install(TARGETS virtual_locator_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME})

ament_package()
