cmake_minimum_required(VERSION 3.16.3)
project(kindr_ros)

if("$ENV{ROS_VERSION}" STREQUAL "1")

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  geometry_msgs
  kindr
  tf
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/RosGeometryMsgPhysicalQuantitiesTest.cpp
    test/RosGeometryMsgRotationTest.cpp
    test/RosGeometryMsgPoseTest.cpp
    test/RosTfPoseTest.cpp
    test/TfConventionTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )
  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        test_${PROJECT_NAME})
  endif(cmake_code_coverage_FOUND)
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

else() # ROS version 2

###############
## Configure ##
###############

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PACKAGE_DEPENDENCIES
  Eigen3
  geometry_msgs
  kindr
  tf2
)

find_package(ament_cmake REQUIRED)
foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(${PROJECT_NAME} INTERFACE
  ${PACKAGE_DEPENDENCIES}
)

#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

##########
## Test ##
##########

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/RosGeometryMsgPhysicalQuantitiesTest.cpp
    test/RosGeometryMsgRotationTest.cpp
    test/RosGeometryMsgPoseTest.cpp
    test/RosTf2PoseTest.cpp
    test/Tf2ConventionTest.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_${PROJECT_NAME}
    ${PACKAGE_DEPENDENCIES}
  )

  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(TEST_BUILD_TARGETS test_${PROJECT_NAME})
  endif(cmake_code_coverage_FOUND)
endif()

ament_package()

endif()
