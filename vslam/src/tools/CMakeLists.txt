cmake_minimum_required(VERSION 2.8.3)
project(tools)

file(GLOB_RECURSE SRC_TOOLS ./*.cpp)
add_library(tools SHARED ${SRC_TOOLS})

INSTALL(TARGETS 
        tools
        DESTINATION ${CMAKE_INSTALL_PREFIX}/lib 
        )

INSTALL(DIRECTORY 
        ${CMAKE_INSTALL_PREFIX}/../include/tools
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
        ) 
