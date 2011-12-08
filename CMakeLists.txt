# Copyright 2011, Thomas Moulard (CNRS/LAAS)
#
# This file is part of sot-motion-planner.
# sot-motion-planner is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-motion-planner is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME sot-motion-planner)
SET(PROJECT_DESCRIPTION "FIXME")
SET(PROJECT_URL "http://github.com/jrl-umi3218/sot-motion-planner")

SET(CUSTOM_HEADER_DIR "sot/motion-planner")

SET(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

SET(CXX_DISABLE_WERROR 1)

SETUP_PROJECT()

# Search for dependencies.
ADD_REQUIRED_DEPENDENCY("jrl-mathtools")

ADD_REQUIRED_DEPENDENCY("jrl-mal >= 1.8.0")

ADD_REQUIRED_DEPENDENCY("dynamic-graph >= 1.0.0")
ADD_REQUIRED_DEPENDENCY("sot-core >= 1.0.0")

ADD_REQUIRED_DEPENDENCY("roboptim-core")

# ViSP
FIND_PROGRAM(VISP_CONFIG visp-config)
IF(NOT VISP_CONFIG)
 MESSAGE(FATAL_ERROR "failed to find visp-config")
ELSE()
 EXEC_PROGRAM("${VISP_CONFIG} --prefix" OUTPUT_VARIABLE VISP_PREFIX)
 EXEC_PROGRAM("${VISP_CONFIG} --cflags" OUTPUT_VARIABLE VISP_CFLAGS)
 EXEC_PROGRAM("${VISP_CONFIG} --libs" OUTPUT_VARIABLE VISP_LIBS)
ENDIF()

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem)
SEARCH_FOR_BOOST()

# walk_interfaces
MACRO(ADD_ROSPACK_DEPENDENCY PKG)
  IF(PKG STREQUAL "")
    MESSAGE(FATAL_ERROR "ADD_ROS_DEPENDENCY invalid call.")
  ENDIF()

  # Transform package name into a valid variable prefix.
  # 1. replace invalid characters into underscores.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" PREFIX "${PKG}")
  # 2. make it uppercase.
  STRING(TOUPPER "${PREFIX}" "PREFIX")

  SET(${PREFIX}_FOUND 0)

  FIND_PROGRAM(ROSPACK rospack)
  IF(NOT ROSPACK)
    MESSAGE(FATAL_ERROR "failed to find the rospack binary. Is ROS installed?")
  ENDIF()

  MESSAGE(STATUS "Looking for ${PKG} using rospack...")
  EXEC_PROGRAM("${ROSPACK} find ${PKG}" OUTPUT_VARIABLE ${PKG}_ROS_PREFIX)
  IF(NOT ${PKG}_ROS_PREFIX)
    MESSAGE(FATAL_ERROR "Failed to detect ${PKG}.")
  ENDIF()

  SET(${PREFIX}_FOUND 1)
  EXEC_PROGRAM("${ROSPACK} export --lang=cpp --attrib=cflags ${PKG}"
    OUTPUT_VARIABLE "${PREFIX}_CFLAGS")
  EXEC_PROGRAM("${ROSPACK} export --lang=cpp --attrib=lflags ${PKG}"
    OUTPUT_VARIABLE "${PREFIX}_LIBS")
ENDMACRO()

MACRO(ROSPACK_USE_DEPENDENCY TARGET PKG)
  IF(PKG STREQUAL "")
    MESSAGE(FATAL_ERROR "ADD_ROS_DEPENDENCY invalid call.")
  ENDIF()

  # Transform package name into a valid variable prefix.
  # 1. replace invalid characters into underscores.
  STRING(REGEX REPLACE "[^a-zA-Z0-9]" "_" PREFIX "${PKG}")
  # 2. make it uppercase.
  STRING(TOUPPER "${PREFIX}" "PREFIX")

  # Make sure we do not override previous flags.
  GET_TARGET_PROPERTY(CFLAGS "${TARGET}" COMPILE_FLAGS)
  GET_TARGET_PROPERTY(LDFLAGS "${TARGET}" LINK_FLAGS)

  # If there were no previous flags, get rid of the XYFLAGS-NOTFOUND
  # in the variables.
  IF(NOT CFLAGS)
    SET(CFLAGS "")
  ENDIF()
  IF(NOT LDFLAGS)
    SET(LDFLAGS "")
  ENDIF()

  # Transform semi-colon seperated list in to space separated list.
  FOREACH(FLAG ${${PREFIX}_CFLAGS})
    SET(CFLAGS "${CFLAGS} ${FLAG}")
  ENDFOREACH()

  FOREACH(FLAG ${${PREFIX}_LDFLAGS})
    SET(LDFLAGS "${LDFLAGS} ${FLAG}")
  ENDFOREACH()

  # Append new flags.
  SET(CFLAGS "${CFLAGS} ${${PREFIX}_CFLAGS}")
  SET(LDFLAGS "${LDFLAGS} ${${PREFIX}_LIBS}")

  # Update the flags.
  SET_TARGET_PROPERTIES(${TARGET}
    PROPERTIES COMPILE_FLAGS "${CFLAGS}" LINK_FLAGS "${LDFLAGS}")
ENDMACRO()

ADD_ROSPACK_DEPENDENCY(walk_interfaces)

# Eigen
FIND_PACKAGE(Eigen REQUIRED)

# Add subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(tests)
ADD_SUBDIRECTORY(bin)
ADD_SUBDIRECTORY(share)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
