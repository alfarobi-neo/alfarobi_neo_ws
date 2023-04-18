cmake_minimum_required(VERSION 3.0.2)
project(alfarobi_vision)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -mavx -mfma)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    sensor_msgs
    geometry_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    robotis_math
    alfarobi_msgs_srvs_actions
)

set(yaml-cpp_DIR /usr/local/lib/cmake/yaml-cpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    # v9_ball_detector
    cfg/v9_ball_detector/BallDetectorParams.cfg
    # v10_goalpost_detector
    cfg/v10_goalpost_detector/GoalpostDetectorParams.cfg
    # v9_localization
    cfg/v9_localization/LocalizationParams.cfg
)

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
    INCLUDE_DIRS include
    CATKIN_DEPENDS
    roscpp
    rospy
    roslib
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
    dynamic_reconfigure
    robotis_math
    alfarobi_msgs_srvs_actions
    DEPENDS OpenCV
    LIBRARIES fitcircle localization_utils v9_amcl
)

###########
## Build ##
###########

#https://www.xuningyang.com/blog/2021-08-21-ROS-multiple-nodes-in-one-package/
# http://wiki.ros.org/catkin/CMakeLists.txt

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/alfarobi_vision.cpp
# )

add_library(${PROJECT_NAME}
    
    # img_source
    src/img_source/img_source.cpp

    # v9_ball_detector
    src/v9_ball_detector/v9_ball_detector.cpp

    # v10_goalpost_detector
    src/v10_goalpost_detector/v10_goalpost_detector.cpp

    # v9_localization
    src/v9_localization/v9_localization.cpp
)

# v9_localization
add_library(v9_amcl STATIC include/v9_localization/v9_amcl.h src/v9_localization/v9_amcl.cpp) 

# vision utils
add_library(fitcircle src/vision_utils/fitcircle.cpp)
add_library(localization_utils src/vision_utils/localization_utils.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}
)

# v9_localization
add_dependencies(v9_amcl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# vision_utils
add_dependencies(fitcircle ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(localization_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# alfarobi_vision_node
# add_executable(alfarobi_vision_node
#     src/alfarobi_vision/alfarobi_vision_node.cpp
# )
# target_link_libraries(alfarobi_vision_node
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBRARIES}
#     ${YAML_CPP_LIBRARIES}
# )
# add_dependencies(alfarobi_vision_node 
#     ${${PROJECT_NAME}_EXPORTED_TARGETS}
#     ${catkin_EXPORTED_TARGETS})

# img_source
add_executable(img_source_node 
    src/img_source/img_source_node.cpp
    src/img_source/img_source.cpp
)
target_link_libraries(img_source_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
add_dependencies(img_source_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})


# v9_ball_detector
add_executable(v9_ball_detector_node 
    src/v9_ball_detector/v9_ball_detector_node.cpp
    src/v9_ball_detector/v9_ball_detector.cpp
)
target_link_libraries(v9_ball_detector_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    fitcircle
)
add_dependencies(v9_ball_detector_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg)

# v10_goalpost_detector
add_executable(v10_goalpost_detector_node 
    src/v10_goalpost_detector/v10_goalpost_detector_node.cpp
    src/v10_goalpost_detector/v10_goalpost_detector.cpp
)
target_link_libraries(v10_goalpost_detector_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
add_dependencies(v10_goalpost_detector_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg)

# v9_localization
add_executable(v9_localization_node 
    src/v9_localization/v9_localization_node.cpp 
    src/v9_localization/v9_localization.cpp
)
target_link_libraries(v9_localization_node
    PUBLIC v9_amcl
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    fitcircle
    localization_utils
)
add_dependencies(v9_localization_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg)

# target_link_libraries(v9_localization_node PUBLIC v9_amcl)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  src/yolov5_detector/yolov5_detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_alfarobi_vision.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)