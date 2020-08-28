cmake_minimum_required(VERSION 3.1.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project (framework)

if(MAVROS)
add_definitions(-DMAVROS=1)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs mavros_msgs clover)
catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs clover )
endif()

if(ROSBOT)
add_definitions(-DROSBOT=1)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs nav_msgs )
catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs )
endif()

if(TURTLESIM)
add_definitions(-DTURTLESIM=1)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  turtlesim
  #message_generation
)
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES beginner_tutorials
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
endif()


include_directories(
  ${catkin_INCLUDE_DIRS}
)

subdirs(main src)
