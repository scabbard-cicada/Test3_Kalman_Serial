cmake_minimum_required(VERSION 3.16)
project(gimbal_serial_driver C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
set(CMAKE_C_STANDARD 11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(interfaces REQUIRED)
find_package(Threads REQUIRED)
# CMake 4 移除了 FindBoost 模块。优先用 Boost 自带的 config 模式，避免开发警告。
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
find_package(Boost REQUIRED)

# ---------------------------------------------------------------
#  下位机固件里与硬件无关的部分（协议、PID、云台控制、GM6020 模型）
#  fake_mcu 直接编译这份代码，所以它的行为和真实固件一致。
#  lower_computer/ 带 COLCON_IGNORE，colcon 不会把它当包扫描
# ---------------------------------------------------------------
set(FW_SHARED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lower_computer/gimbal_fw/Shared)
if(NOT EXISTS ${FW_SHARED_DIR}/gimbal_proto.c)
  message(FATAL_ERROR "找不到下位机共享代码：${FW_SHARED_DIR}")
endif()
add_library(gimbal_fw_shared STATIC
  ${FW_SHARED_DIR}/gimbal_proto.c
  ${FW_SHARED_DIR}/pid.c
  ${FW_SHARED_DIR}/gm6020.c
  ${FW_SHARED_DIR}/gimbal_ctrl.c)
target_include_directories(gimbal_fw_shared PUBLIC ${FW_SHARED_DIR})
set_target_properties(gimbal_fw_shared PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(gimbal_fw_shared PUBLIC m)

# ---------------------------------------------------------------
#  上位机协议库（C++，独立于下位机的 C 实现）
# ---------------------------------------------------------------
add_library(gimbal_protocol STATIC src/protocol.cpp)
target_include_directories(gimbal_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(gimbal_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------
#  串口驱动节点（组件 + 独立可执行文件）
# ---------------------------------------------------------------
add_library(gimbal_serial_driver_component SHARED
  src/asio_serial.cpp
  src/serial_driver_node.cpp)
target_include_directories(gimbal_serial_driver_component PRIVATE include ${Boost_INCLUDE_DIRS})
target_link_libraries(gimbal_serial_driver_component
  gimbal_protocol
  rclcpp::rclcpp
  rclcpp_components::component
  ${interfaces_TARGETS}
  Threads::Threads)
rclcpp_components_register_node(gimbal_serial_driver_component
  PLUGIN gimbal_serial_driver::SerialDriverNode
  EXECUTABLE gimbal_serial_driver_node)

# ---------------------------------------------------------------
#  软件下位机
# ---------------------------------------------------------------
add_executable(fake_mcu_node src/fake_mcu_node.cpp)
target_link_libraries(fake_mcu_node
  gimbal_fw_shared
  rclcpp::rclcpp
  ${std_srvs_TARGETS}
  util
  Threads::Threads)

install(TARGETS gimbal_serial_driver_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS fake_mcu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------
#  测试：上下位机两份协议实现互相编解码 + 黄金字节向量
# ---------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol gimbal_protocol gimbal_fw_shared)
endif()

ament_package()
