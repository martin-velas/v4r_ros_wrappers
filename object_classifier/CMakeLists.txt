cmake_minimum_required(VERSION 2.8.3)
project(object_classifier)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs visualization_msgs segmentation_srv_definitions classifier_srv_definitions object_perception_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)
find_package( V4R REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES object_classifier
   CATKIN_DEPENDS segmentation_srv_definitions classifier_srv_definitions
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include
 #${FAAT_PCL_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
  ${V4R_INCLUDE_DIRS}
)

set(incs include/pcl_conversions.h include/segmenter.h include/global_nn_classifier_ros.h)
## Declare a cpp library
# add_library(object_classifier
#   src/${PROJECT_NAME}/object_classifier.cpp
# )

## Declare a cpp executable
add_executable(object_classifier_node src/global_nn_classifier_ros.cpp ${incs})
add_executable(object_classifier_demo_node src/test.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(object_classifier_demo_node ${catkin_EXPORTED_TARGETS} ${STRANDSV4R_LIBS})
add_dependencies(object_classifier_node ${catkin_EXPORTED_TARGETS} ${STRANDSV4R_LIBS})

FIND_PACKAGE(Ceres)
IF(${Ceres_FOUND})
  INCLUDE_DIRECTORIES(${CERES_INCLUDES})
ELSE(${Ceres_FOUND})
  MESSAGE("\nATTENTION: Ceres solver is not installed!\n")
ENDIF(${Ceres_FOUND})

## Specify libraries to link a library or executable target against
 target_link_libraries(object_classifier_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
   ${V4R_LIBS}
  ${CERES_LIBRARIES}

)

target_link_libraries(object_classifier_demo_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
   ${V4R_LIBS}
  ${CERES_LIBRARIES}
  )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS object_classifier object_classifier_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_classifier.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
