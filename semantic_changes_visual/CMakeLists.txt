## BEGIN_TUTORIAL
## This CMakeLists.txt file for rviz_plugin_tutorials builds both the TeleopPanel tutorial and the ImuDisplay tutorial.
##
## First start with some standard catkin stuff.
cmake_minimum_required(VERSION 2.8.3)
project(semantic_changes_visual)
find_package(catkin REQUIRED COMPONENTS rviz message_generation eigen_conversions tf_conversions)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

find_package(PCL 1.7 REQUIRED)

find_package(V4R REQUIRED)

find_package( OpenCV REQUIRED )

include_directories(include
    ${PCL_INCLUDE_DIRS}
    ${V4R_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

add_message_files(
  FILES 
   MoveChange.msg
   SimpleChange.msg
   ChangedScene.msg
   ObjectDetection.msg
   ObjectLabel.msg
)

add_service_files(
  FILES
   get_removed_objects.srv
   reset_change_detection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  semantic_changes_visual
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime sensor_msgs eigen_conversions
)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/semantic_changes_visual/change_display.h
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/change_display.cpp
  src/bb_display.cpp
  ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${V4R_LIBS})
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

install(DIRECTORY include/semantic_changes_visual
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)

add_executable(test_publisher src/test_publisher.cpp)
add_dependencies(test_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_publisher
   ${V4R_LIBS}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_executable(change_detection src/change_detection.cpp)
add_dependencies(change_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(change_detection
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
   ${OpenCV_LIBS}
)

add_executable(test_change_detection src/test_change_detection.cpp)
add_dependencies(test_change_detection ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(test_change_detection
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${V4R_LIBS}
   ${OpenCV_LIBS}
)
