cmake_minimum_required(VERSION 2.8.3)
project(endonasal_teleop)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf_conversions
  message_generation
  message_runtime
  std_msgs
)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(QT_USE_QTXML 1)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder

add_message_files(
	FILES 
	matrix8.msg
	matrix6.msg
	config3.msg
	vector7.msg
	kinout.msg
#	cannula3def.msg
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   getStartingConfig.srv
   getStartingKin.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 	# Or other packages containing msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  LIBRARIES endonasal_teleop
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime message_generation
)

#include(${QT_USE_FILE})


###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${HDAPI_HDU_INCLUDE_DIR}
  ${HDAPI_INCLUDE_DIR}
  ${HLAPI_HLU_INCLUDE_DIR}
  ${HLAPI_INCLUDE_DIR}

  ${catkin_INCLUDE_DIRS}
  /usr/include/OGRE
  include
  /home/remireaa/Documents/LIBRARIES/Boost
  /home/remireaa/Documents/LIBRARIES/Eigen
  /home/remireaa/Documents/LIBRARIES/MathTools
  /home/remireaa/Documents/LIBRARIES/CannulaKinematics
  /home/remireaa/Documents/LIBRARIES/RapidXML
  /home/remireaa/Documents/TeleopLeap
  /home/remireaa/Documents/Qt
  /home/remireaa/catkin_ws/src
#  /home/remireaa/my_library/ros_lib/tf
)

link_directories(
  /home/remireaa/Documents/LIBRARIES/Boost
  /home/remireaa/Documents/LIBRARIES/Eigen
  /home/remireaa/Documents/LIBRARIES/MathTools
  /home/remireaa/Documents/LIBRARIES/CannulaKinematics
  /home/remireaa/Documents/LIBRARIES/RapidXML
  /home/remireaa/Documents/TeleopLeap
  /home/remireaa/Documents/Qt
  /home/remireaa/catkin_ws/src
#  /home/remireaa/my_library/ros_lib/tf
)

set(SRC
  src/kinematics.cpp
  src/workspace_display.cpp
#  src/main.cpp
  src/needle_broadcaster.cpp
#  src/needle_display.cpp
  src/resolved_rates.cpp
  src/resolved_rates_position.cpp
  src/tf_broadcaster.cpp
#  src/motorTest.cpp
)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)


## Declare a C++ library
# add_library(endonasal_teleop
#   src/${PROJECT_NAME}/endonasal_teleop.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(endonasal_teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
# Find the QtWidgets library
find_package(Qt5Widgets)
find_package(Qt5PrintSupport)
find_package(Qt5Core)
find_package(Qt5Gui)


## Declare a C++ executable
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#add_executable(needle_display src/needle_display.cpp)
add_executable(needle_broadcaster src/needle_broadcaster.cpp)
add_executable(kinematics src/kinematics.cpp)
add_executable(workspace_display src/workspace_display.cpp)
add_executable(resolved_rates src/resolved_rates.cpp)
add_executable(resolved_rates_position src/resolved_rates_position.cpp)
#add_executable(motorTest src/motorTest.cpp)
#add_executable(main src/main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(endonsasal_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
target_link_libraries(needle_broadcaster ${catkin_LIBRARIES})
#target_link_libraries(needle_display ${catkin_LIBRARIES})
target_link_libraries(kinematics ${catkin_LIBRARIES} CannulaKinematics)
target_link_libraries(workspace_display ${catkin_LIBRARIES} CannulaKinematics)
target_link_libraries(resolved_rates ${catkin_LIBRARIES} CannulaKinematics)
target_link_libraries(resolved_rates_position ${catkin_LIBRARIES} CannulaKinematics)
#target_link_libraries(main ${catkin_LIBRARIES} CannulaKinematics)

target_link_libraries(kinematics Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})
target_link_libraries(workspace_display Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})
target_link_libraries(resolved_rates Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})
target_link_libraries(resolved_rates_position Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})
#target_link_libraries(main Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})

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
# install(TARGETS endonasal_teleop endonasal_teleop_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_endonasal_teleop.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
