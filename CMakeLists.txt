cmake_minimum_required(VERSION 3.10)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-modules")

set(PROJECT_NAME test)
project(${PROJECT_NAME})
set(EXECUTABLE ${PROJECT_NAME})


set(CMAKE_CXX_STANDARD 17)

find_package(ortools REQUIRED)
set(LIBS ${ORTOOLS_LIBRARIES})
include_directories(${ORTOOLS_INCLUDE_DIRS})

add_executable(${EXECUTABLE} main.cc ${SRC_FILES} ${INC_FILES} ${TEMPLATE_FILES}) 
target_link_libraries(${EXECUTABLE} ${LIBS})

