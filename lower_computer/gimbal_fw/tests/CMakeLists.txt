cmake_minimum_required(VERSION 3.16)
project(gimbal_fw_host_tests C)

# 主机端单元测试：只编译 Shared/，不依赖 HAL / 交叉编译器
#   cmake -S tests -B build/host-tests && cmake --build build/host-tests
#   ctest --test-dir build/host-tests --output-on-failure

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(SHARED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Shared)

add_library(gimbal_shared STATIC
  ${SHARED_DIR}/gimbal_proto.c
  ${SHARED_DIR}/pid.c
  ${SHARED_DIR}/gm6020.c
  ${SHARED_DIR}/gimbal_ctrl.c)
target_include_directories(gimbal_shared PUBLIC ${SHARED_DIR})
target_compile_options(gimbal_shared PRIVATE -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion)
target_link_libraries(gimbal_shared PUBLIC m)

add_executable(test_shared test_shared.c)
target_compile_options(test_shared PRIVATE -Wall -Wextra)
target_compile_definitions(test_shared PRIVATE _DEFAULT_SOURCE)
target_link_libraries(test_shared PRIVATE gimbal_shared)

enable_testing()
add_test(NAME shared COMMAND test_shared)
