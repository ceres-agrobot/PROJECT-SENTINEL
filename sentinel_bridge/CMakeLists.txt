cmake_minimum_required(VERSION 3.8)

project(sentinel_bridge
    VERSION 1.0.0
    DESCRIPTION "sentinel bridge package cmake configuration"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(
        -Wall
        -Wextra
        -Wpedantic
        -Werror=return-type
    )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sentinel_interfaces REQUIRED)

add_executable(sentinel_state_adapter_node src/sentinel_state_adapter_node.cpp)
    ament_target_dependencies(sentinel_state_adapter_node
    rclcpp sensor_msgs nav_msgs ros_gz_interfaces sentinel_interfaces
)

install(TARGETS sentinel_state_adapter_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
