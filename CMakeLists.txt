cmake_minimum_required(VERSION 3.0.2)
project(test_autodock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  geometry_msgs
  tf
  tf2_msgs
  tf2_ros
  actionlib_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_action_files(
  DIRECTORY action
  FILES MPC.action
)


catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    rospy
    std_msgs
    tf
    tf2_ros
    nav_msgs
    actionlib_msgs
    message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_mpc src/mpc_planner_ros.cpp
                                src/mpc_planner.cpp
                                src/trajectory.cpp)
add_executable(mpc_planner_node src/mpc_planner_node.cpp)
target_link_libraries(mpc_planner_node ${PROJECT_NAME}_mpc ${catkin_LIBRARIES} ipopt)


#############
## Install ##
#############

# Install targets
install(TARGETS 
        ${PROJECT_NAME}_mpc
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)