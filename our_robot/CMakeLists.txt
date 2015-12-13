cmake_minimum_required(VERSION 2.8.3)
project(our_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

add_action_files(
  DIRECTORY action_server/action
  FILES FollowJointTrajectory.action
)#action_server需要的

find_package(Boost REQUIRED system filesystem date_time thread)

#generate_messages(
 # DEPENDENCIES actionlib_msgs std_msgs 
#)#action_server需要的# Or other packages containing msgs

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES our_robot ????????
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
#actionlib_msgs#action_server需要的
)
find_package(Eigen REQUIRED)

###########
## Build ##
###########


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(kdl_rviz)
add_subdirectory(qt_publish)
add_subdirectory(rviz_subscribe)
add_subdirectory(sub_jointState_pub_moveJ_6)

