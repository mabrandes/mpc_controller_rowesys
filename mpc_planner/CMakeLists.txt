cmake_minimum_required(VERSION 2.8.3)
project(mpc_planner)

find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lib
#  CATKIN_DEPENDS packages
#  DEPENDS system_lib
)

## mark src for installation
install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
  PATTERN ".svn" EXCLUDE
)