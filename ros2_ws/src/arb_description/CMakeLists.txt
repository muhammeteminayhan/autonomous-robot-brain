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

# The robot model reads its dimensions from the repo-level configs/ directory,
# which is the single source of truth shared by the URDF, the C++ nodes and the
# Python evaluation tools. Those files are copied into the package share at
# install time so that at runtime xacro can resolve them through
# $(find arb_description)/config/... like any other package resource -- rather
# than through a fragile relative path out of the install tree.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../configs/robot.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../configs/sensors.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../configs/world.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
