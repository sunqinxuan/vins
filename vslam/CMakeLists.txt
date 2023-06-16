cmake_minimum_required(VERSION 2.8.3)
project(vslam)

SET(CMAKE_BUILD_TYPE "Debug") #Debug/Release
MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall -fPIC -O3  -Wno-error=deprecated-declarations -Wno-deprecated-declarations -g") #-g

add_compile_options(-std=c++14)
add_definitions(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

#option(USE_SIM "option for cmake" on) 
#option(USE_ARM "option for arm  " off)
#option(USE_CUDA "option for cuda" off)
#option(USE_CODE "option for cmake" on) 

#if(USE_SIM)
#   message("USE_SIM true")
#   add_definitions(-DUSE_SIM)
#endif()
#if(USE_ARM)
#   message("USE_ARM true")
#   add_definitions(-DARM)
#   cmake_policy(SET CMP0046 OLD)
#endif()
#if(USE_CUDA)
#   find_package(CUDA QUIET REQUIRED)
#   set(CUDA_PROPAGATE_HOST_FLAGS OFF)
#   SET(CUDA_HOST_COMPILER /usr/bin/g++)
#   add_definitions(-DCUDA_FOUND)
#   LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")
#   cuda_include_directories(${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc)
#   cuda_add_library(cuda_filter ${CUDA_INCLUDE_HS} ${CUDA_SRCS} )
#   include_directories(${CUDA_INCLUDE_DIRS} )
#   #list(APPEND ALL_TARGET_LIBRARIES ${CUDA_LIBRARIES})
#   #list(APPEND ALL_TARGET_LIBRARIES cuda_filter )
#endif()


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

include_directories(include ${catkin_INCLUDE_DIRS}) #include

#load thrid party library
#link_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib)
#set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/include)

#yaml
#include_directories(${THIRD_PARTY_DIR})#/yaml-cpp)

# add_library(yaml-cpp SHARED IMPORTED)
# set_target_properties(yaml-cpp PROPERTIES IMPORTED_LOCATION 
# 		${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib/libyaml-cpp.so)

#pcl
#include_directories(${THIRD_PARTY_DIR}/pcl-1.10)
#file(GLOB_RECURSE PCL_LIBRARIES "third_party/lib/libpcl*")

#eigen
#include_directories(${THIRD_PARTY_DIR}/eigen3)

#geographic
#include_directories(${THIRD_PARTY_DIR}/Geographic)

#ndt_omp
#include_directories(${THIRD_PARTY_DIR}/ndt_omp)

#ceres
#include_directories(${THIRD_PARTY_DIR}/ceres)

#sophus
#include_directories(${THIRD_PARTY_DIR}/sophus)


#if(USE_CODE)
   #message("USE_CODE : true")
   #message("load function by code library!")
   # include_directories(include)
   set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/lib)

   add_subdirectory(${PROJECT_SOURCE_DIR}/src/tools)
   add_subdirectory(${PROJECT_SOURCE_DIR}/src/dataflow)
   add_subdirectory(${PROJECT_SOURCE_DIR}/src/frontend)
   add_subdirectory(${PROJECT_SOURCE_DIR}/src/camera_models)

   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/models)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/sensor_data)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/publisher)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/subscriber)

   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/data_pretreat)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/mapping/loop_closing)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/mapping/back_end)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/mapping/mapping)
   #add_subdirectory(${PROJECT_SOURCE_DIR}/src/matching)
	 #else()
	 #   message("USE_CODE : false")
	 #   message("load function by share library!")
	 #   link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/lib)
	 #   include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/include)
	 #endif()



add_executable(slam_node src/node/slam_node.cpp)
add_dependencies(slam_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(slam_node dataflow frontend ${catkin_LIBRARIES})

#add_executable(front_end_node src/apps/front_end_node.cpp)
#add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(front_end_node mapping_frontend ${catkin_LIBRARIES})

#add_executable(back_end_node src/apps/back_end_node.cpp)
#add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
#target_link_libraries(back_end_node mapping_backend ${catkin_LIBRARIES})

#add_executable(loop_closing_node src/apps/loop_closing_node.cpp)
#add_dependencies(loop_closing_node ${catkin_EXPORTED_TARGETS} searchLoop_gencpp)
#target_link_libraries(loop_closing_node mapping_loopclosing ${catkin_LIBRARIES})

#add_executable(mapping_node src/apps/mapping_node.cpp)
#add_dependencies(mapping_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
#target_link_libraries(mapping_node mapping_mapping ${catkin_LIBRARIES})

#add_executable(matching_node src/apps/matching_node.cpp)
#add_dependencies(matching_node ${catkin_EXPORTED_TARGETS} pubMap_gencpp)
#target_link_libraries(matching_node matching ${catkin_LIBRARIES})

