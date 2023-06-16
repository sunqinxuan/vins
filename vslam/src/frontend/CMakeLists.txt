cmake_minimum_required(VERSION 2.8.3)
project(frontend)

#find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3)

list(APPEND LIB_FRONTEND 
	${catkin_LIBRARIES} ${OpenCV_LIBS} 
	tools dataflow camera_models)

file(GLOB_RECURSE SRC_FRONTEND ./*.cpp)
add_library(frontend SHARED ${SRC_FRONTEND})
target_link_libraries(frontend ${LIB_FRONTEND})

INSTALL(TARGETS frontend 
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib 
        )

INSTALL(DIRECTORY 
        ${CMAKE_INSTALL_PREFIX}/../include/frontend
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
        ) 



