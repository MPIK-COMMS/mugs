# 
# Copyright (c) 2013, 2016 Max Planck Institute for Biological Cybernetics
# All rights reserved.
# 
# This file is part of MUGS - Mobile and Unrestrained Gazetracking Software.
#
# MUGS is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# MUGS is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with MUGS.  If not, see <http://www.gnu.org/licenses/>. 
#

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of 
build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
Release RelWithDebInfo MinSizeRel.")
ELSE()
SET(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
ENDIF()

# CMake modules/macros are in a subfolder
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

project(MUG)

# search for all required packages
find_package(Eigen REQUIRED)
find_package(LibGp REQUIRED)
find_package(Boost COMPONENTS system thread timer filesystem REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${LIBGP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty)

set(SOURCE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/head_eye_transform.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gaze_tracker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/screen_model_flat.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/eye_model_linear.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/eye_model_moore.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/eye_model_gp.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/eye_model_gp_optimization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/lsl_interface.cpp
    )

# create target library
add_library (mug  ${SOURCE})
target_link_libraries(mug
    ${LIBGP_LIBRARIES}
    ${Boost_THREAD_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY} 
    ${Boost_TIMER_LIBRARY}
    )
install(TARGETS mug
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    )
