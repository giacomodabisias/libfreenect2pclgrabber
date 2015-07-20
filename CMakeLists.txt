cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 
set(CMAKE_PREFIX_PATH /prefix)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(CUDA 7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_PREFIX_PATH}/include/libfreenect2/tinythread/)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLEWF_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

#find_library(FREENECT2 NAMES freenect2)
find_library(GLFW NAMES glfw)

add_definitions(-DGLEW_MX -DGLEW_STATIC)

add_executable(Kinect2Grabber test.cpp )

target_link_libraries(Kinect2Grabber ${CUDA_LIBRARIES} ${OpenCV_LIBS} ${FREENECT2} ${GLFW} ${PCL_LIBRARIES})

