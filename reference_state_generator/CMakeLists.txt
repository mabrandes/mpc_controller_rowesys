cmake_minimum_required(VERSION 3.0.2)
project(reference_state_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)



find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  States_mpc.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  
)

catkin_package(
    CATKIN_DEPENDS roscpp rospy message_runtime std_msgs 
)

catkin_install_python(PROGRAMS
  src/ref_state_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
