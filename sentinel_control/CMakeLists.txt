cmake_minimum_required(VERSION 3.8)

project(sentinel_control
    VERSION 1.0.0
    DESCRIPTION "sentinel control package cmake configuration"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)   # Important to find all libraries and global variables outside the workspace.

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sentinel_interfaces REQUIRED)

add_library(sentinel_gait_model STATIC src/gait_model.cpp)
target_include_directories(sentinel_gait_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(sentinel_locomotion_node src/sentinel_locomotion_node.cpp)
target_link_libraries(sentinel_locomotion_node sentinel_gait_model)
ament_target_dependencies(sentinel_locomotion_node rclcpp geometry_msgs sensor_msgs std_msgs sentinel_interfaces)

install(TARGETS sentinel_locomotion_node 
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS sentinel_gait_model 
    ARCHIVE DESTINATION lib
)

install(DIRECTORY include/ 
    DESTINATION include
)
install(DIRECTORY config launch 
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(sentinel_gait_model)
ament_package()