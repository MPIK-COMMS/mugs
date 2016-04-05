#
# This file is part of the MUG - Mobile and Unrestrained Gazetracking software package.
#
# Copyright (c) 2013, Max Planck Institute for Biological Cybernetics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its 
#       contributors may be used to endorse or promote products derived from 
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
