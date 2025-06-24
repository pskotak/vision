cmake_minimum_required(VERSION 3.11)

project(vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

find_package( OpenCV REQUIRED )
set(DEPENDENCIES ${DEPENDENCIES} realsense2)

set(SOURCE_FILES
    vision.cpp
    main.cpp    
)

add_executable(vision ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(vision ${DEPENDENCIES} ${OpenCV_LIBS})

install(TARGETS vision RUNTIME DESTINATION bin)
