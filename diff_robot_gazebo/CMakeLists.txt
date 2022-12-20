cmake_minimum_required(VERSION 3.0.2)
project(diff_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  tf
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(diff_interface_plugin
  src/DiffInterfacePlugin.cpp
  src/MotorSim.cpp
)
target_link_libraries(diff_interface_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

# Extract *.tar.gz files in the mesh folder
FILE(GLOB files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/meshes" "${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.tar.gz")
FOREACH(filename ${files})
  MESSAGE(STATUS "Extracting file: ${filename}")
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf ${filename}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  )
ENDFOREACH(filename)

### Install Rules
install(
  TARGETS
    diff_interface_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY
    launch
    meshes
    urdf
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN *.tar.gz EXCLUDE
)
