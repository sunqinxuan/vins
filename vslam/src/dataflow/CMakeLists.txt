cmake_minimum_required(VERSION 2.8.3)
project(dataflow)

#find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")
find_package(Eigen3)
#include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

list(APPEND LIB_DATAFLOW ${catkin_LIBRARIES} ${OpenCV_LIBS} tools)
		 #publisher subscriber sensor_data tools)

file(GLOB_RECURSE SRC_DATAFLOW ./*.cpp)
add_library(dataflow SHARED ${SRC_DATAFLOW})
target_link_libraries(dataflow ${LIB_DATAFLOW})

INSTALL(TARGETS dataflow
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib 
        )

INSTALL(DIRECTORY 
        ${CMAKE_INSTALL_PREFIX}/../include/dataflow
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
        ) 

