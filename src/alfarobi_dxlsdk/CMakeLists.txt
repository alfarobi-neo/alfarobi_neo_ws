################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(alfarobi_dxlsdk)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS roscpp rospy)
find_package(yaml-cpp REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy
  LIBRARIES alfarobi_dxlsdk #dynamixel_sdk
)


################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


if(APPLE)
  add_library(dynamixel_sdk
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_mac.cpp
  )
else()
  add_library(dynamixel_sdk
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_linux.cpp
  )
endif()

# add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES})

# ## Declare a C++ library
# add_library(servo_controller #${PROJECT_NAME}
#   src/alfarobi_dxlsdk/servo_controller.cpp
# )
# target_link_libraries(alfarobi_dxlsdk ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} yaml-cpp)
add_library(${PROJECT_NAME}
  src/alfarobi_dxlsdk/servo_controller.cpp

)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(main
  src/main.cpp
  src/alfarobi_dxlsdk/servo_controller.cpp
  src/dynamixel_sdk/packet_handler.cpp
  src/dynamixel_sdk/protocol1_packet_handler.cpp
  src/dynamixel_sdk/protocol2_packet_handler.cpp
  src/dynamixel_sdk/group_sync_read.cpp
  src/dynamixel_sdk/group_sync_write.cpp
  src/dynamixel_sdk/group_bulk_read.cpp
  src/dynamixel_sdk/group_bulk_write.cpp
  src/dynamixel_sdk/port_handler.cpp
  src/dynamixel_sdk/port_handler_linux.cpp
)
target_link_libraries(main
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    
    )
################################################################################
# Install
################################################################################
# install(TARGETS dynamixel_sdk
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# catkin_install_python(PROGRAMS
#   src/dynamixel_sdk/group_bulk_read.py
#   src/dynamixel_sdk/group_bulk_write.py
#   src/dynamixel_sdk/group_sync_read.py
#   src/dynamixel_sdk/group_sync_write.py
#   src/dynamixel_sdk/packet_handler.py
#   src/dynamixel_sdk/port_handler.py
#   src/dynamixel_sdk/protocol1_packet_handler.py
#   src/dynamixel_sdk/protocol2_packet_handler.py
#   src/dynamixel_sdk/robotis_def.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

################################################################################
# Test
################################################################################
