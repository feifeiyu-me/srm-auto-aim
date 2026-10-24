cmake_minimum_required(VERSION 3.8)
project(srm_armor_refiner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(srm_aim_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(armor_refinement src/refiner.cpp)
target_compile_features(armor_refinement PUBLIC cxx_std_17)
target_include_directories(armor_refinement PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(armor_refinement PUBLIC ${OpenCV_LIBS})
target_compile_options(armor_refinement PRIVATE -Wall -Wextra -Wpedantic)

add_executable(armor_refiner_node src/armor_refiner_node.cpp)
target_link_libraries(armor_refiner_node armor_refinement)
ament_target_dependencies(armor_refiner_node rclcpp rcl_interfaces std_msgs sensor_msgs
  diagnostic_msgs cv_bridge srm_aim_interfaces)
target_compile_options(armor_refiner_node PRIVATE -Wall -Wextra -Wpedantic)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_refiner test/test_refiner.cpp)
  target_link_libraries(test_refiner armor_refinement)
  ament_add_gtest(test_frame_matcher test/test_frame_matcher.cpp)
  target_compile_features(test_frame_matcher PRIVATE cxx_std_17)
  target_include_directories(test_frame_matcher PRIVATE include)
  ament_target_dependencies(test_frame_matcher std_msgs sensor_msgs srm_aim_interfaces)
endif()

install(TARGETS armor_refinement EXPORT armor_refinement_export
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS armor_refiner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(armor_refinement_export HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenCV std_msgs sensor_msgs srm_aim_interfaces)
ament_package()
