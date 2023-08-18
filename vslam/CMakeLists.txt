cmake_minimum_required(VERSION 2.8.3)
project(vslam)

SET(CMAKE_BUILD_TYPE "Debug") #Debug/Release
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall -fPIC -O3  -Wno-error=deprecated-declarations -Wno-deprecated-declarations -g") #-g


add_compile_options(-std=c++14)
add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf
  #message_generation 
  #std_srvs
)

#add_service_files(
#   FILES
#   saveMap.srv
#   pubMap.srv
#   optimizeMap.srv
#   searchLoop.srv
#)

#add_message_files(
#   DIRECTORY msg
#   FILES
#   InsInfo.msg
#   PcInsInfo.msg
#   KeyFrameFusion.msg
#)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
   std_msgs
   geometry_msgs
   sensor_msgs
)

include_directories(src ${catkin_INCLUDE_DIRS}) #include
include_directories(src/camera_models ${catkin_INCLUDE_DIRS}) #include

set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/lib)

add_subdirectory(${PROJECT_SOURCE_DIR}/src/tools)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/camera_models)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/optimizer)

add_subdirectory(${PROJECT_SOURCE_DIR}/src/dataflow)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/vio)
add_subdirectory(${PROJECT_SOURCE_DIR}/src/loopfusion)

add_executable(vio_node src/vio_node.cpp)
add_dependencies(vio_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vio_node dataflow vio ${catkin_LIBRARIES})

add_executable(loopfusion_node src/loopfusion_node.cpp)
add_dependencies(loopfusion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loopfusion_node dataflow loopfusion ${catkin_LIBRARIES})

add_executable(kitti2msg src/kitti2msg.cpp)
add_dependencies(kitti2msg ${catkin_EXPORTED_TARGETS})
target_link_libraries(kitti2msg ${catkin_LIBRARIES} ${OpenCV_LIBS})
