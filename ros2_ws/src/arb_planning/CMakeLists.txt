cmake_minimum_required(VERSION 3.16)
project(arb_planning VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(arb_msgs REQUIRED)
find_package(arb_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/grid_planner.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}_core arb_core)

add_executable(global_planner_node src/global_planner_node.cpp)
target_link_libraries(global_planner_node ${PROJECT_NAME}_core)
ament_target_dependencies(global_planner_node
  rclcpp nav_msgs geometry_msgs arb_msgs arb_core tf2 tf2_ros)
add_executable(planner_benchmark src/planner_benchmark.cpp)
target_link_libraries(planner_benchmark ${PROJECT_NAME}_core)
ament_target_dependencies(planner_benchmark rclcpp nav_msgs arb_core)

install(TARGETS global_planner_node planner_benchmark DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_core
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_grid_planner test/test_grid_planner.cpp)
  target_link_libraries(test_grid_planner ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp nav_msgs arb_core)
ament_package()
