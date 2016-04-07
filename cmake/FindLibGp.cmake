# - Try to finde LibGp
# Once done this will define
#    LIBGP_FOUND - System has LibGp
#    LIBGP_INCLUDE_DIRS - The LibGp include directories
#    LIBGP_LIBRARIES - The libraries needed to use LibGp
#    LIBGP_DEFINITIONS - Compiler switches required for using LibGp
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

find_package(PkgConfig)
pkg_check_modules(PC_LIBGP QUIET libgp)
set(LIBGP_DEFINITIONS ${PC_LIBGP_CFLAGS_OTHER})

find_path(LIBGP_INCLUDE_DIR gp.h
          HINTS ${PC_LIBGP_INCLUDEDIR} ${PC_LIBGP_INCLDE_DIRS}
          PATH_SUFFIXES gp)

find_library(LIBGP_LIBRARY names gp
             HINTS ${PC_LIBGP_LIBDIR} ${PC_LIBGP_INCLUDE_DIRS}
             PATH_SUFFIXES gp)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBGP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(LibGp DEFAULT_MSG
                                  LIBGP_LIBRARY LIBGP_INCLUDE_DIR)

mark_as_advanced(LIBGP_INCLUDE_DIR LIBGP_LIBRARY)

set(LIBGP_INCLUDE_DIRS ${LIBGP_INCLUDE_DIR})
set(LIBGP_LIBRARIES ${LIBGP_LIBRARY})
