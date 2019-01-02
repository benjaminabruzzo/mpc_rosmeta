cmake_minimum_required(VERSION 2.8.3)
project(mayataka_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS cmake_modules
	roscpp
	rospy
	std_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	# LIBRARIES nmpc_model
#  CATKIN_DEPENDS roscpp rospy std_msgs
	DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# add_library(nmpc_model src/nmpc_model.cpp)


add_executable(mayataka_nmpc src/mayataka_nmpc.cpp)
	target_link_libraries(mayataka_nmpc ${catkin_LIBRARIES})
	# target_link_libraries(mayataka_nmpc ${catkin_LIBRARIES} nmpc_model)
		
	# target_link_libraries(mayataka_nmpc simulator)
	# 	add_library(simulator STATIC src/simulator.cpp src/numerical_integrator.cpp)
	# target_link_libraries(mayataka_nmpc nmpcsolver)
	# 	add_library(nmpcsolver STATIC src/matrixfree_gmres.cpp src/init_cgmres.cpp src/continuation_gmres.cpp src/multiple_shooting_cgmres.cpp)
		
