cmake_minimum_required(VERSION 2.8.3)
project(tools)

file(GLOB_RECURSE SRC_TOOLS ./*.cpp)
add_library(tools SHARED ${SRC_TOOLS})
