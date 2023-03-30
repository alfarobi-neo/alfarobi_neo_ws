cmake_minimum_required(VERSION 2.8.3)
project(arduino_controller)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    roslib
    sensor_msgs
    geometry_msgs
    eigen_conversions
    robotis_math
    eigen_conversions
    tf
    tf2)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)

set(Qt5_DIR "qt5/lib/cmake/Qt5")
find_package(Qt5 REQUIRED Gui Widgets SerialPort Core)
find_package(Eigen3 REQUIRED) 
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arduino_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 ${catkin_INCLUDE_DIRS}
 ${YAML_CPP_INCLUDE_DIR}
)


add_executable(${PROJECT_NAME}_node
    arduino_controller_node.cpp
    controller.cpp
    )

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${Eigen3_LIBRARIES}
   ${YAML_CPP_LIBRARIES}
   Qt5::Gui Qt5::Core Qt5::SerialPort Qt5::Widgets
 )

