cmake_minimum_required(VERSION 2.8.3)
project(kindr_ros)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs tf
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
#  LIBRARIES kindr_ros
  CATKIN_DEPENDS geometry_msgs tf
  DEPENDS eigen
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

#############
## Testing ##
#############

catkin_add_gtest(${PROJECT_NAME}-test 
    test/test_main.cpp
    test/RosGeometryMsgPhysicalQuantitiesTest.cpp
    test/RosGeometryMsgRotationTest.cpp
    test/RosGeometryMsgPoseTest.cpp
    test/RosTfPoseTest.cpp
    test/TfConventionTest.cpp
)
if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
