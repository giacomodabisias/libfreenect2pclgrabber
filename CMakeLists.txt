cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

set(DEPTH_PATH ${CMAKE_CURRENT_SOURCE_DIR}/depth_registration/)
set(DEPTH_INCLUDE_DIR ${DEPTH_PATH}/include)
set(DEPTH_SOURCE_DIR ${DEPTH_PATH}/src)

find_package(CUDA REQUIRED)
find_package( OpenCV REQUIRED )

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CUDA_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${LIBFREENECT2_INCLUDE_DIRS} )
include_directories( ${GLEW_INCLUDE_DIRS})
include_directories( ${GLEWF_INCLUDE_DIRS})
include_directories( ${DEPTH_INCLUDE_DIR})

ADD_DEFINITIONS(-DGLEW_MX -DGLEW_STATIC)


set(CUDA_PROPAGATE_HOST_FLAGS OFF) 

if(${APPLE})
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-ccbin /usr/bin/clang)
else()
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-ccbin /usr/bin/gcc -arch=sm_30 )
set(EXTRALIB rt)
endif()

add_executable(Kinect2Grabber test.cpp ${DEPTH_SOURCE_DIR}/depth_registration.cpp ${DEPTH_SOURCE_DIR}/depth_registration_cpu.cpp )

target_link_libraries(Kinect2Grabber ${CUDA_LIBRARIES} ${EXTRALIB} ${OpenCV_LIBS} ${LIBFREENECT2_LIBS} ${GLEW_LIBS} ${GLEWF_LIBS} ${PCL_LIBRARIES})

