cmake_minimum_required(VERSION 3.16)
project(arb_description VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  meshes
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
