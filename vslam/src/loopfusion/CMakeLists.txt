cmake_minimum_required(VERSION 2.8.3)
project(loopfusion)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

list(APPEND LIB_LOOPFUSION 
	${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} 
	tools dataflow camera_models optimizer)

file(GLOB_RECURSE SRC_LOOPFUSION ./*.cpp)
add_library(loopfusion SHARED ${SRC_LOOPFUSION})
target_link_libraries(loopfusion ${LIB_LOOPFUSION})



