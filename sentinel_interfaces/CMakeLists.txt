cmake_minimum_required(VERSION 3.8)

project(sentinel_interfaces
    VERSION 1.0.0
    DESCRIPTION "Sentinel Interfaces package cmake configuration"
    LANGUAGES C CXX
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
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Contacts.msg"
    "msg/ContactsStamped.msg"
    "msg/Imu.msg"
    "msg/Joints.msg"
    "msg/PID.msg"
    "msg/Point.msg"
    "msg/PointArray.msg"
    "msg/Pose.msg"
    "msg/Velocities.msg"
    DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
