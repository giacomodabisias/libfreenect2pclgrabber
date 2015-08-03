cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber C CXX)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(HAVE_OPENMP)
endif()

find_package(Threads)
find_package(GLEW)

find_package( OpenCV REQUIRED )

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CUDA_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${GLEW_INCLUDE_DIRS})
include_directories( ${GLEWF_INCLUDE_DIRS})
include_directories(${LIBFREENECT2_DIRS})


find_path( LIBFREENECT2_INCLUDE_DIR 
    libfreenect2/libfreenect2.hpp
  PATHS 
    /usr/local/include
    $ENV{LIBFREENECT2_ROOT}/include
)

find_library( LIBFREENECT2_LIBRARIES
  NAMES freenect2
  PATHS
  $ENV{LIBFREENECT2_ROOT}/lib
  ${OPENGL_LIBRARY_DIR}
  /usr/local/lib
)

find_path( LIBFREENECT2_TINYTHREAD_INCLUDE_DIR
    NAMES tinythread.h
    PATHS
    /usr/local/include/libfreenect2/tinythread/

)

set(LIBFREENECT2_INCLUDE_DIRS
    ${LIBFREENECT2_INCLUDE_DIR}
    ${LIBFREENECT2_TINYTHREAD_INCLUDE_DIR}
)

include_directories(${LIBFREENECT2_INCLUDE_DIRS})

#include_directories(/usr/local/include/pcl-1.7/)

#add_definitions(-DGLEW_MX -DGLEW_STATIC)

add_executable(Kinect2Grabber test.cpp )

target_link_libraries(Kinect2Grabber ${CMAKE_THREAD_LIBS_INIT} ${CUDA_LIBRARIES} ${OpenCV_LIBS} ${LIBFREENECT2_LIB} ${GLEW_LIBRARIES} ${GLFW_LIB} ${PCL_LIBRARIES} ${LIBFREENECT2_LIBRARIES})

#cmake .. -DGLEW_LIB=/home/gf/Libraries/libfreenect2/depends/glew/lib64/libGLEW.so -DGLFW_LIB=/home/gf/Libraries/libfreenect2/examples/protonect/lib/libglfw.so.3.0 -DGLEW_INCLUDE_DIRS=/home/gf/Libraries/libfreenect2/depends/glew/include -DGLEWF_INCLUDE_DIRS=/home/gf/Libraries/libfreenect2/depends/glfw/include -DLIBFREENECT2_LIB=/home/gf/Libraries/libfreenect2/examples/protonect/lib/libfreenect2.so