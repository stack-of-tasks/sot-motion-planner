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

# Search for dependencies.
# Boost
SET(BOOST_COMPONENTS filesystem)
SEARCH_FOR_BOOST()

# Add subdirectories.
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(tests)
ADD_SUBDIRECTORY(bin)
ADD_SUBDIRECTORY(share)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
