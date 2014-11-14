cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

find_package(CUDA REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${CUDA_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${LIBFREENECT2_INCLUDE_DIRS} )
include_directories( ${GLEW_INCLUDE_DIRS})
include_directories( ${GLEWF_INCLUDE_DIRS})
ADD_DEFINITIONS(-DGLEW_MX -DGLEW_STATIC)


set(CUDA_PROPAGATE_HOST_FLAGS OFF) 

if(${APPLE})
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-ccbin /usr/bin/clang)
else()
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-ccbin /usr/bin/gcc -arch=sm_30 )
set(EXTRALIB rt)
endif()

add_executable(Kinect2Grabber test.cpp)

target_link_libraries(Kinect2Grabber ${CUDA_LIBRARIES} ${EXTRALIB} ${OpenCV_LIBS} ${LIBFREENECT2_LIBS} ${GLEW_LIBS} ${GLEWF_LIBS})

