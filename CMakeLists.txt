cmake_minimum_required(VERSION 3.8)
project(hybridAStar)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp hybrid_breadth_first.h hybrid_breadth_first.cpp)
add_executable(hybridAStar ${SOURCE_FILES})