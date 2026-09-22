cmake_minimum_required(VERSION 3.16)
project(arb_localization VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arb_msgs REQUIRED)
find_package(arb_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Estimation logic with no rclcpp dependency, so it can be unit tested without
# a node, an executor or a graph.
add_library(${PROJECT_NAME}_core SHARED
  src/differential_drive_odometry.cpp
  src/ekf.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}_core arb_core)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
target_link_libraries(wheel_odometry_node ${PROJECT_NAME}_core)
ament_target_dependencies(wheel_odometry_node
  rclcpp sensor_msgs geometry_msgs nav_msgs arb_core tf2 tf2_ros tf2_geometry_msgs)

add_executable(ekf_node src/ekf_node.cpp)
target_link_libraries(ekf_node ${PROJECT_NAME}_core)
ament_target_dependencies(ekf_node
  rclcpp sensor_msgs geometry_msgs nav_msgs arb_msgs arb_core
  tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS wheel_odometry_node ekf_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry ${PROJECT_NAME}_core)

  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp nav_msgs arb_core tf2_ros)
ament_package()
