cmake_minimum_required(VERSION 2.8.3)
project(optimizer)

#find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})



list(APPEND LIB_FACTOR 
	${catkin_LIBRARIES}  ${CERES_LIBRARIES})
	#${OpenCV_LIBS} gtsam
	#tools dataflow camera_models)

file(GLOB_RECURSE SRC_FACTOR ./*.cpp)

add_library(optimizer SHARED ${SRC_FACTOR})
target_link_libraries(optimizer ${LIB_FACTOR})




