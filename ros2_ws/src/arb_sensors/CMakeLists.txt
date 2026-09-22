cmake_minimum_required(VERSION 3.16)
project(arb_sensors VERSION 0.1.0 LANGUAGES CXX)

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

# --- processing library -------------------------------------------------
# The signal processing is a plain library with no rclcpp dependency, so the
# unit tests can exercise it without spinning a node or standing up a graph.
# Every node below is a thin wiring shim around it.
add_library(${PROJECT_NAME}_core SHARED
  src/imu_processor.cpp
  src/encoder_processor.cpp
  src/lidar_processor.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(${PROJECT_NAME}_core sensor_msgs arb_core)

# --- nodes ---------------------------------------------------------------
foreach(node imu_preprocessor encoder_preprocessor lidar_preprocessor)
  add_executable(${node}_node src/${node}_node.cpp)
  target_link_libraries(${node}_node ${PROJECT_NAME}_core)
  ament_target_dependencies(${node}_node rclcpp sensor_msgs arb_msgs arb_core)
  install(TARGETS ${node}_node DESTINATION lib/${PROJECT_NAME})
endforeach()

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
  ament_add_gtest(test_noise_model test/test_noise_model.cpp)
  target_link_libraries(test_noise_model ${PROJECT_NAME}_core)

  ament_add_gtest(test_encoder_processor test/test_encoder_processor.cpp)
  target_link_libraries(test_encoder_processor ${PROJECT_NAME}_core)

  ament_add_gtest(test_imu_processor test/test_imu_processor.cpp)
  target_link_libraries(test_imu_processor ${PROJECT_NAME}_core)

  ament_add_gtest(test_lidar_processor test/test_lidar_processor.cpp)
  target_link_libraries(test_lidar_processor ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rclcpp sensor_msgs arb_msgs arb_core)
ament_package()
