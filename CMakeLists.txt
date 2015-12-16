# Copyright 2015, Giacomo Dabisias"
#
# Dual licensed with permission under GPLv3 or later and the 2 clause simplified BSD.
# https://github.com/giacomodabisias/libfreenect2pclgrabber/issues/10
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# @Author 
# Giacomo Dabisias, PhD Student
# PERCRO, (Laboratory of Perceptual Robotics)
# Scuola Superiore Sant'Anna
# via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy

cmake_minimum_required(VERSION 2.8)
Project (Kinect2Grabber CXX)

OPTION(ENABLE_OPENCL "Enable OpenCL support" ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x") 

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Freenect2 REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${GLFW_INCLUDE_DIRS})
include_directories(${FREENECT2_INCLUDE_DIRS})

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

