cmake_minimum_required(VERSION 2.8.3)
project(grasper)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  mar_perception
  mar_robot
  mar_robot_arm5e
  mar_ros_bridge
  message_generation
  roscpp
  rospy
  std_msgs
  std_srvs
  tf
)


add_action_files(
 DIRECTORY action
 FILES turnValve.action unplugHotStab.action plugHotStab.action
 )



generate_messages(
   DEPENDENCIES
  actionlib_msgs  std_msgs
)


catkin_package(

  CATKIN_DEPENDS actionlib actionlib_msgs mar_perception mar_robot mar_robot_arm5e mar_ros_bridge message_generation roscpp rospy std_msgs std_srvs tf

)



include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)


add_library(grasper src/joint_offset.cpp)



add_executable(turn_valve_server src/turn_valve_server.cpp)
target_link_libraries( turn_valve_server grasper ${catkin_LIBRARIES})

add_executable(turn_valve_client src/turn_valve_client.cpp)
target_link_libraries( turn_valve_client grasper ${catkin_LIBRARIES})

add_executable(hot_stab_server src/hot_stab_server.cpp)
target_link_libraries( hot_stab_server grasper ${catkin_LIBRARIES})

add_executable(hot_stab_client src/hot_stab_client.cpp)
target_link_libraries( hot_stab_client grasper ${catkin_LIBRARIES})



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
# install(TARGETS grasper grasper_node
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_grasper.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
