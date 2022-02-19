cmake_minimum_required(VERSION 3.15)
project(kindr_ros)

# Use C++14 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kindr REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${kindr_INCLUDE_DIRS})

add_library(${PROJECT_NAME} INTERFACE)

set_target_properties(${PROJECT_NAME} PROPERTIES
  PUBLIC_HEADER
  kindr_ros/kindr_ros.hpp
  kindr_ros/RosGeometryMsgPhysicalQuantities.hpp
  kindr_ros/RosGeometryMsgPose.hpp
  kindr_ros/RosGeometryMsgRotation.hpp
  kindr_ros/RosGeometryMsgTwist.hpp
  kindr_ros/RosTfPose.hpp
)

install(
  TARGETS
    ${PROJECT_NAME}
  PUBLIC_HEADER
    DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/RosGeometryMsgPhysicalQuantitiesTest.cpp
    test/RosGeometryMsgRotationTest.cpp
    test/RosGeometryMsgPoseTest.cpp
    test/RosTfPoseTest.cpp
    test/TfConventionTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  # TODO(SivertHavso): add back code coverage:
  # if(TARGET ${PROJECT_NAME}-test)
  #   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
  #   find_package(cmake_code_coverage QUIET)
  #   if(cmake_code_coverage_FOUND)
  #     add_gtest_coverage(TEST_BUILD_TARGETS ${PROJECT_NAME}-test)
  #   endif()
  # endif()
endif()

ament_export_include_directories(include ${EIGEN3_INCLUDE_DIR})

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
