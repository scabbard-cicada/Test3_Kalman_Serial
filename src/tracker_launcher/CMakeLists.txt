cmake_minimum_required(VERSION 3.16)
project(tracker_launcher)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config foxglove
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/verify_ekf.py
  scripts/verify_gimbal.py
  scripts/verify_serial.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
