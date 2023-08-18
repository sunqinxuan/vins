cmake_minimum_required(VERSION 2.8.3)
project(vio)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

list(APPEND LIB_VIO 
	${catkin_LIBRARIES} ${OpenCV_LIBS}	
	tools dataflow camera_models optimizer)

file(GLOB_RECURSE SRC_VIO ./*.cpp)
add_library(vio SHARED ${SRC_VIO})
target_link_libraries(vio ${LIB_VIO})




