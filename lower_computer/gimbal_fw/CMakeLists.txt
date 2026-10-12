cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME gimbal_fw)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 不接电机、只用 C 板时：cmake --preset Debug -DGIMBAL_VIRTUAL_MOTOR=ON
option(GIMBAL_VIRTUAL_MOTOR "Run a simulated GM6020 on the board instead of driving CAN" OFF)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 应用层：与 HAL 打交道的部分
    ${CMAKE_CURRENT_SOURCE_DIR}/App/app.c
    # 与硬件无关的部分：协议、PID、云台控制、GM6020 编解码。主机测试和 fake_mcu 编译的是同一份
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/gimbal_proto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/gm6020.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/gimbal_ctrl.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/App
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    GIMBAL_VIRTUAL_MOTOR=$<BOOL:${GIMBAL_VIRTUAL_MOTOR}>
)

# 应用层代码开严格警告；CubeMX 生成的代码不动
set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/App/app.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/gimbal_proto.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/pid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/gm6020.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Shared/gimbal_ctrl.c
    PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wdouble-promotion;-Werror")

# 生成 .hex / .bin，便于用 STM32CubeProgrammer 或 DFU 烧录
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex / .bin")

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
