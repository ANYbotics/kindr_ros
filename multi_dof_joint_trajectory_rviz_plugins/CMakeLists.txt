## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(multi_dof_joint_trajectory_rviz_plugins)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
   rviz
   trajectory_msgs
   rqt_gui_cpp
)

catkin_package(
 INCLUDE_DIRS
   include
 LIBRARIES
   ${PROJECT_NAME}
 CATKIN_DEPENDS
   rviz
   trajectory_msgs
   rqt_gui_cpp
)

link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
set(multi_dof_joint_trajectory_rviz_plugins_HDRS
  include/multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryDisplay.hpp
  include/multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointConnectionVisual.hpp
  include/multi_dof_joint_trajectory_rviz_plugins/MultiDOFJointTrajectoryPointVisual.hpp
)

set(multi_dof_joint_trajectory_rviz_plugins_SRCS
  src/MultiDOFJointTrajectoryDisplay.cpp
  src/MultiDOFJointTrajectoryPointConnectionVisual.cpp
  src/MultiDOFJointTrajectoryPointVisual.cpp
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(multi_dof_joint_trajectory_rviz_plugins_MOCS
    ${multi_dof_joint_trajectory_rviz_plugins_HDRS}
  )
else()
  qt4_wrap_cpp(multi_dof_joint_trajectory_rviz_plugins_MOCS
    ${multi_dof_joint_trajectory_rviz_plugins_HDRS}
  )
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  ${multi_dof_joint_trajectory_rviz_plugins_SRCS}
  ${multi_dof_joint_trajectory_rviz_plugins_MOCS}
)
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES} ${catkin_LIBRARIES}
)
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()



# Specify dependencies on automatically generated service files
#add_dependencies(${PROJECT_NAME})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
## END_TUTORIAL

## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(DIRECTORY media/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

#install(PROGRAMS scripts/send_test_msgs.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

