cmake_minimum_required(VERSION 3.5)
project(launchers)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY
        ${CMAKE_CURRENT_SOURCE_DIR}/config
        ${CMAKE_CURRENT_SOURCE_DIR}/launch
        ${CMAKE_CURRENT_SOURCE_DIR}/shellscript
        ${CMAKE_CURRENT_SOURCE_DIR}/systemd
    DESTINATION share/${PROJECT_NAME}
)

install(
    PROGRAMS
        scripts/l2_throttle_joy_remap.py
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
