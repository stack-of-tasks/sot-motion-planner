# Copyright 2011, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of sot-motion-planner. sot-motion-planner is free
# software: you can redistribute it and/or modify it under the terms
# of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# sot-motion-planner is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# Look for private headers and libraries.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/src)


# SOT_MOTION_PLANNER_TEST(NAME)
# -----------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(SOT_MOTION_PLANNER_TEST NAME)
  ADD_EXECUTABLE(${NAME} ${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  # Add dependencies.
  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mathtools)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)
#  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)


  # Link against Boost.
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_LIBRARIES})

  TARGET_LINK_LIBRARIES(${NAME} localizer)
  TARGET_LINK_LIBRARIES(${NAME} feet-follower)
  #TARGET_LINK_LIBRARIES(${NAME} visp)
ENDMACRO(SOT_MOTION_PLANNER_TEST)

# Time related tools.
SOT_MOTION_PLANNER_TEST(time)
