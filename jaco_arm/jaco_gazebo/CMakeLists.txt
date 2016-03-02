cmake_minimum_required(VERSION 2.8.3)
project(jaco_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  jaco_description
  sensor_msgs
  jaco_joints
  roslint
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    # can check with if (TARGET doc) whether the target 'doc' was already used for another package.
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(jaco_gazebo_doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)



## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


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
   INCLUDE_DIRS include gz2_support
   LIBRARIES jaco_gazebo
   CATKIN_DEPENDS gazebo_ros jaco_description roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
  
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  gz2_support
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(JOINT_CONTROL_SRC
   src/JointController.cpp
   gz2_support/JointControllerNewImpl.cpp
   src/JacoGazeboJointControl.cpp
   src/JacoGazeboJointControlLocalImpl.cpp
   src/JacoGazeboJointControlLocalImplRegistration.cpp)


## Declare a C++ library
add_library(jaco_gazebo_joint_control
    ${JOINT_CONTROL_SRC} 
)

add_library(jaco_gazebo_joint_state_client
   src/JointController.cpp
   gz2_support/JointControllerNewImpl.cpp
   src/JacoGazeboJointStateClient.cpp
)

add_library(jaco_gazebo_joint_state_publisher
   src/JacoGazeboJointStatePublisher.cpp
)

add_library(jaco_gazebo_joint_trajectory_server
   src/JacoGazeboJointTrajectoryServer.cpp
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(jaco_gazebo_joint_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_gazebo_joint_state_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_gazebo_joint_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(jaco_gazebo_joint_trajectory_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(test_joint_trajectory_client 
    test/SimpleJointTrajectoryClient.cpp
)


add_executable(test_set_joint_state_publisher_vel 
    test/SetJointStatePublisher.cpp
)

add_executable(test_set_joint_state_publisher_pos
    test/SetJointStatePositionPublisher.cpp
)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(test_joint_trajectory_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_set_joint_state_publisher_vel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_set_joint_state_publisher_pos ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

set (DEP_LIBS
   ${GAZEBO_LIBRARIES}   
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}   
)

target_link_libraries(jaco_gazebo_joint_control
   ${DEP_LIBS}
)

target_link_libraries(jaco_gazebo_joint_state_client
   ${DEP_LIBS}
)

target_link_libraries(jaco_gazebo_joint_state_publisher
   ${DEP_LIBS}
)


target_link_libraries(jaco_gazebo_joint_trajectory_server
   ${DEP_LIBS}
)


target_link_libraries(test_joint_trajectory_client 
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}   
)


target_link_libraries(test_set_joint_state_publisher_vel 
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}   
)

target_link_libraries(test_set_joint_state_publisher_pos 
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}   
)


# style checking
roslint_cpp()

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
# install(TARGETS jaco_gazebo jaco_gazebo_node
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

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)
    install(DIRECTORY 
        ${CMAKE_CURRENT_BINARY_DIR}/docs/html 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
)
endif (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)

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
