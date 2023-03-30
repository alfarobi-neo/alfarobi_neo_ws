cmake_minimum_required(VERSION 2.8.3)
project(model_based_control)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    cmake_modules
    eigen_conversions
    roscpp 
    rospy 
    roslib
    std_msgs 
)

find_package(Eigen3 REQUIRED)
set(yaml-cpp_DIR "/usr/local/lib/cmake/yaml-cpp")
find_package(yaml-cpp REQUIRED)

## Declare ROS messages and services



## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    roscpp 
    cmake_modules
)

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES model_based_control
    LIBRARIES ${PROJECT_NAME}
    )

include_directories(
    include
    #model_based_control
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )

## Generate added messages and services
#generate_messages(DEPENDENCIES
 #   std_msgs
#)

add_library(${PROJECT_NAME}
    src/csvManager.cpp
    src/densis.cpp
    src/modelBasedControl.cpp
    src/servoObserver.cpp
    )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${YAML_CPP_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Eigen3_LIBRARIES}
    )

#target_link_libraries(model_based_control_exec 
#    ${YAML_CPP_LIBRARIES}
#    ${catkin_LIBRARIES}
#)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
