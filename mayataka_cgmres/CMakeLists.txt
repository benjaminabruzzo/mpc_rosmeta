cmake_minimum_required(VERSION 3.1)
project(nmpc_solver CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

include_directories(${CMAKE_SOURCE_DIR})
include_directories(src)
include_directories(include)

add_executable(sim src/main.cpp)

target_link_libraries(sim simulator)
target_link_libraries(sim nmpcsolver)
target_link_libraries(sim model)

add_library(nmpcsolver 	STATIC 
	src/matrixfree_gmres.cpp 
	src/init_cgmres.cpp 
	src/continuation_gmres.cpp 
	src/multiple_shooting_cgmres.cpp)
add_library(simulator STATIC 
	src/simulator.cpp 
	src/numerical_integrator.cpp)
add_library(model STATIC 
	src/nmpc_model.cpp)