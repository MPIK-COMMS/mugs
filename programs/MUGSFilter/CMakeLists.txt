# 
# Copyright (c) 2013, 2017 Max Planck Institute for Biological Cybernetics
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

project(MugsFilter)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake)

find_package(Eigen REQUIRED)
find_package(LibGp REQUIRED)
find_package(Boost COMPONENTS system thread timer filesystem program_options REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${LIBGP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

link_directories(/usr/local/lib)

add_executable(mugs_filter ${CMAKE_CURRENT_SOURCE_DIR}/mugs_filter.cpp)
target_link_libraries(mugs_filter
    /usr/local/lib/libmug.a
    ${LIBGP_LIBRARIES} 
    ${Boost_THREAD_LIBRARY}
    ${Boost_FILESYSTEM_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY} 
    ${Boost_TIMER_LIBRARY} 
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    )

