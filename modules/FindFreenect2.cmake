# Copyright 2015, Giacomo Dabisias"
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
#
# - Try to find Freenect2
# Once done this will define
#
#  FREENECT2_FOUND - system has Freenect2
#  FREENECT2_INCLUDE_DIRS - the Freenect2 include directory
#  FREENECT2_LIBRARY - Link these to use Freenect2
#  FREENECT2_LIBRARIES



find_path(FREENECT2_INCLUDE_DIRS NAMES libfreenect2.hpp
	HINTS
	/usr/local/include/libfreenect2/
	/usr/include/libfreenect2
	/usr/local/include/
	/usr/include/
	}
)
 
find_library(FREENECT2_LIBRARY NAMES freenect2)

if(FREENECT2_INCLUDE_DIRS AND FREENECT2_LIBRARY)
  set(FREENECT2_FOUND TRUE)
endif()

if(FREENECT2_LIBRARY)
    set(FREENECT2_LIBRARY ${FREENECT2_LIBRARY})
endif()

if (FREENECT2_FOUND)
  mark_as_advanced(FREENECT2_INCLUDE_DIRS FREENECT2_LIBRARY FREENECT2_LIBRARIES)
endif()
