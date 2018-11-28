cmake_minimum_required(VERSION 2.8.3)

project(endonasal_teleop)

#add_definitions(-std=c++11)
add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
	medlab_motor_control_board
	roscpp
  rospy
  tf
  tf_conversions
  message_generation
  message_runtime
  std_msgs
)

##find_package(Qt5Core REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core)
##find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
##find_package(Qt5PrintSupport)
##find_package(Qt5Widgets)
##find_package(Qt5Gui)

#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(QT_USE_QTXML 1)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES 
	matrix8.msg
	matrix6.msg
	config3.msg
	vector7.msg
	kinout.msg
	OmniState.msg
)

# Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   getStartingConfig.srv
 #  getStartingKin.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 	# Or other packages containing msgs
 )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES endonasal_teleop
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime	
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
##  ${HDAPI_HDU_INCLUDE_DIR}
## ${HDAPI_INCLUDE_DIR}
##  ${HLAPI_HLU_INCLUDE_DIR}
##  ${HLAPI_INCLUDE_DIR}

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
#  /home/remireaa/catkin_ws/src
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
#  /home/remireaa/catkin_ws/src
#  /home/remireaa/my_library/ros_lib/tf
)

#set(SRC
##  src/kinematics.cpp
#  src/workspace_display.cpp
#  src/needle_broadcaster.cpp
##  src/needle_display.cpp
##  src/resolved_rates.cpp
#  src/tf_broadcaster.cpp
#	src/RoboticsMath.cpp
#  src/CTR3Robot.cpp
##  src/ResolvedRatesController.cpp
#  src/endonasal_teleop.cpp
#)

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



## Declare a C++ executable
add_executable(endonasal_teleop src/endonasal_teleop.cpp 
																src/RoboticsMath.cpp 
																src/CTR3Robot.cpp 
																src/ResolvedRatesController.cpp
																src/endonasalTeleopFSM.cpp
																src/PhantomOmniRos.cpp
																src/mcbros.cpp
)

##target_link_libraries(endonasal_teleop ${catkin_LIBRARIES} CannulaKinematics)
target_link_libraries(endonasal_teleop Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES} CannulaKinematics)


#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#add_executable(needle_broadcaster src/needle_broadcaster.cpp)
add_executable(workspace_display src/workspace_display.cpp)

##add_executable(resolved_rates src/resolved_rates.cpp)
##add_executable(kinematics src/kinematics.cpp)
##add_executable(needle_display src/needle_display.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(endonsasal_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
#target_link_libraries(needle_broadcaster ${catkin_LIBRARIES})
target_link_libraries(workspace_display ${catkin_LIBRARIES} CannulaKinematics)


##target_link_libraries(kinematics ${catkin_LIBRARIES} CannulaKinematics)
##target_link_libraries(resolved_rates ${catkin_LIBRARIES} CannulaKinematics)
##target_link_libraries(needle_display ${catkin_LIBRARIES})


target_link_libraries(workspace_display Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})


##target_link_libraries(resolved_rates Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})
##target_link_libraries(kinematics Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui ${catkin_LIBRARIES})
