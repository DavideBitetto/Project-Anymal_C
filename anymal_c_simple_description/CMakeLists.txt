cmake_minimum_required(VERSION 3.5.1)
project(anymal_c_simple_description)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

find_package(catkin REQUIRED COMPONENTS
controller_interface
hardware_interface
pluginlib
roscpp
rospy
cmake_modules
)

SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")

FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_BUILD_TYPE Release)

catkin_package(
CATKIN_DEPENDS
controller_interface
hardware_interface
message_runtime
pluginlib
roscpp
DEPENDS Eigen
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(floating_lib src/floating_base_controller.cpp)

target_link_libraries(floating_lib ${catkin_LIBRARIES}
                                          ${RBDL_LIBRARY}
                                  ${RBDL_URDFReader_LIBRARY})

add_executable(set_command src/set_command_FB.cpp)
target_link_libraries(set_command ${catkin_LIBRARIES})

add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES})

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      roslaunch
  )
  roslaunch_add_file_check(launch/load.launch)
endif()
