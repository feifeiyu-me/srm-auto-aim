cmake_minimum_required(VERSION 3.8)
project(srm_aim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keypoint2D.msg"
  "msg/ArmorDetection.msg"
  "msg/ArmorDetectionArray.msg"
  "msg/ArmorLocation.msg"
  "msg/ArmorLocationArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gtest(test_armor_messages test/test_armor_messages.cpp)
  if(TARGET test_armor_messages)
    target_compile_features(test_armor_messages PRIVATE cxx_std_17)
    ament_target_dependencies(test_armor_messages rclcpp)
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(test_armor_messages "${cpp_typesupport_target}")
  endif()

  # Generated Python bindings and their shared libraries are in the build tree.
  ament_add_pytest_test(test_python_messages test/test_python_messages.py
    APPEND_ENV
      "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    TIMEOUT 60
  )
endif()

ament_package()
