# 
# Copyright (c) 2017 Max Planck Institute for Biological Cybernetics
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

enable_testing()
find_package(GTest REQUIRED)
FIND_PACKAGE(Threads REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

# add sources
set(MUGS_TESTS
    all_tests.cpp
    preprocessing_test.cpp
    sample_test.cpp
    )
    
# create test
add_executable(mugstest ${MUGS_TESTS})
target_link_libraries(mugstest
        mug
        ${CMAKE_THREAD_LIBS_INIT}
        ${GTEST_BOTH_LIBRARIES}
        )