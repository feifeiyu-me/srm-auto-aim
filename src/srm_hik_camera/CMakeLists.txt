cmake_minimum_required(VERSION 3.8)
project(srm_hik_camera)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(HIK_INCLUDE_DIR
    "$ENV{HOME}/sp_vision_camera/io/hikrobot/include"
    CACHE PATH "HikRobot SDK header directory")

find_library(HIK_LIBRARY
    NAMES MvCameraControl
    PATHS /opt/MVS/lib/64
    NO_DEFAULT_PATH
    REQUIRED)

if(NOT EXISTS "${HIK_INCLUDE_DIR}/MvCameraControl.h")
    message(FATAL_ERROR "Missing MvCameraControl.h in ${HIK_INCLUDE_DIR}")
endif()

add_executable(hik_camera_node src/hik_camera_node.cpp)

target_include_directories(hik_camera_node PRIVATE
    ${HIK_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(hik_camera_node
    rclcpp sensor_msgs cv_bridge)

target_link_libraries(hik_camera_node
    ${HIK_LIBRARY}
    ${OpenCV_LIBS})

set_target_properties(hik_camera_node PROPERTIES
    INSTALL_RPATH "/opt/MVS/lib/64")

install(TARGETS hik_camera_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
