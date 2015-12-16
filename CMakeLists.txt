cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber CXX)

OPTION(ENABLE_OPENCL "Enable OpenCL support" ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLEWF_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${GLFW_INCLUDE_DIRS})

find_package(freenect2 REQUIRED)

include_directories(${freenect2_INCLUDE_DIRS})


if(ENABLE_OPENCL)
  find_package(OpenCL)

  if(OpenCL_FOUND)
    add_definitions("-DHAVE_OPENCL")
  endif(OpenCL_FOUND)

endif(ENABLE_OPENCL)

add_executable(Kinect2Grabber test.cpp)

target_link_libraries(Kinect2Grabber ${OpenCV_LIBS} ${freenect2_LIBRARIES} ${GLFW_LIBRARIES} ${PCL_LIBRARIES})

