cmake_minimum_required(VERSION 2.8.3)
project(jaco_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  jaco_joints
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

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
#   INCLUDE_DIRS include
   #LIBRARIES
   CATKIN_DEPENDS jaco_joints
   #   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#  include
  #${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_joint_trajectory_client
    test/SimpleJointTrajectoryClient.cpp
)
add_executable(test_set_joint_state_publisher_vel
    test/SetJointStatePublisher.cpp
)
add_executable(test_set_joint_state_publisher_pos
    test/SetJointStatePositionPublisher.cpp
)

add_dependencies(test_joint_trajectory_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_set_joint_state_publisher_vel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_set_joint_state_publisher_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set (DEP_LIBS
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(test_joint_trajectory_client
    ${DEP_LIBS}
)

target_link_libraries(test_set_joint_state_publisher_vel
   ${DEP_LIBS}
)

target_link_libraries(test_set_joint_state_publisher_pos
   ${DEP_LIBS}
)

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_jaco_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
