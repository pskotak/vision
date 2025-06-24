cmake_minimum_required(VERSION 3.0)

project(vision)

add_executable(vision main.cpp)

install(TARGETS vision RUNTIME DESTINATION bin)
