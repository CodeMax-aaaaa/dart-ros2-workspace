CMAKE_MINIMUM_REQUIRED(VERSION 3.16)
PROJECT(libsocketcancpp)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/include/exceptions)

add_library(libsocketcancpp STATIC src/CanDriver.cpp)