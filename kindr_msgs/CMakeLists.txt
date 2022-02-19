cmake_minimum_required(VERSION 3.5.1)
project(kindr_msgs)

find_package(ament_cmake_auto)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VectorAtPosition.msg"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
