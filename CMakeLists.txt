cmake_minimum_required(VERSION 2.8.3)
project(dyros_mobile_manipulator_controllers)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(. SRC_LIST)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/FindRBDL.cmake)
INCLUDE(cmake/ide.cmake)

find_package(PkgConfig QUIET)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  franka_hw
  franka_control
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  roslib
)

set(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(Eigen3 REQUIRED)
find_package(Franka 0.5.0 REQUIRED)
# find_package(qpOASES REQUIRED)
catkin_package(
  LIBRARIES dyros_mobile_manipulator_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    franka_hw
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  DEPENDS Franka
)

###########
## Build ##include
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  # ${QPOASES_INCLUDE_DIR}
  ${RBDL_INCLUDE_DIR}

)


add_library(dyros_mobile_manipulator_controllers
  src/hqp_wholebody_controller.cpp
)

add_dependencies(dyros_mobile_manipulator_controllers
  ${${PROJECT_NAME}_EXPORTEHQPD_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)



SET(${PROJECT_NAME}_TASKS_HEADERS
    include/tasks/task-base.h
    include/tasks/task-com.h
    include/tasks/task-joint-bounds.h
    include/tasks/task-joint-posture.h
    include/tasks/task-mobile.h
    include/tasks/task-motion.h
    include/tasks/task-operational.h
    include/tasks/task-singularity.h
    include/tasks/trajectory-base.h
)
SET(${PROJECT_NAME}_CONSTRAINT_HEADERS
    include/constraint/constraint-base.h
    include/constraint/constraint-bound.h
    include/constraint/constraint-equality.h
    include/constraint/constraint-inequality.h
    include/constraint/fwd_constraints.h
)
SET(${PROJECT_NAME}_CONTACT_HEADERS
    include/contacts/contact-base.h
    include/contacts/contact-3d.h
)
SET(${PROJECT_NAME}_TRAJECTORY_HEADERS
    include/trajectories/trajectory-base.h
    include/trajectories/trajectory-jointspace.h
    include/trajectories/trajectory-operationalspace.h
)
SET(${PROJECT_NAME}_SOLVERS_HEADERS
#    include/solvers/eiquadprog_2011.h
    include/solvers/fwd_solver.h
    include/solvers/solver-HQP-base.h
    include/solvers/solver-HQP-eiquadprog.h
    include/solvers/solver-HQP-factory.h
    include/solvers/solver-HQP-factory.hxx
    include/solvers/solver-HQP-output.h
    include/solvers/solver-HQP-qpoases.h
    include/solvers/solver-utils.h
)
SET(${PROJECT_NAME}_CONTROLLER_HEADERS
    include/controller/Inverse-dynamics.h
)
SET(${PROJECT_NAME}_ROBOT_HEADERS
    include/robot/robot_model.h
)
SET(${PROJECT_NAME}_UTILS_HEADERS
    include/utils/container.h
    include/utils/motion.h
    include/utils/utils.h
)

SET(HEADERS
    include/fwd.h
    ${${PROJECT_NAME}_TASKS_HEADERS}
    ${${PROJECT_NAME}_CONSTRAINT_HEADERS}
    ${${PROJECT_NAME}_CONTACT_HEADERS}
    ${${PROJECT_NAME}_TRAJECTORY_HEADERS}
    ${${PROJECT_NAME}_SOLVERS_HEADERS}
    ${${PROJECT_NAME}_CONTROLLER_HEADERS}
    ${${PROJECT_NAME}_UTILS_HEADERS}
    ${${PROJECT_NAME}_ROBOT_HEADERS}       
)
LIST(REMOVE_DUPLICATES HEADERS)

SET(HEADERS_FULL_PATH "")
FOREACH(header ${HEADERS})
  LIST(APPEND HEADERS_FULL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${header}") 
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  GET_FILENAME_COMPONENT(headerPath ${header} PATH)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/${header}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${headerPath}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header ${HEADERS})

ADD_SUBDIRECTORY(src)



target_link_libraries(dyros_mobile_manipulator_controllers PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  hqp_controller
  ${RBDL_LIBRARY}
)
#############
## Install ##
#############

install(TARGETS dyros_mobile_manipulator_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES dyros_mobile_manipulator_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(
  PROGRAMS scripts/interactive_marker.py scripts/move_to_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dyros_mobile_manipulator_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
