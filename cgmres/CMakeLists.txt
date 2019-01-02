cmake_minimum_required(VERSION 2.8.3)
project(cgmres)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cgmres
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	../cgmres_mayataka
	${catkin_INCLUDE_DIRS}
)


add_executable(         cgmres_model  src/cgmres_model.cpp)
  # add_dependencies(     cgmres_model  hast_gencpp)
  target_link_libraries(cgmres_model  ${catkin_LIBRARIES})
