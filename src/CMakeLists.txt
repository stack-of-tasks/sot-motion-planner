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

INCLUDE(../cmake/python.cmake)

FUNCTION(COMPILE_PLUGIN NAME SOURCES ENTITIES)
  ADD_LIBRARY(${NAME} SHARED ${SOURCES})
  SET_TARGET_PROPERTIES(${lib} PROPERTIES
    PREFIX ""
    SOVERSION ${PROJECT_VERSION})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} dynamic-graph)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} sot-core)

  INSTALL(TARGETS ${NAME} DESTINATION lib/plugin)

  # build python submodule
  STRING(REPLACE - _ PYTHON_LIBRARY_NAME ${NAME})
  ADD_DEPENDENCIES(${NAME} MKDIR_${PYTHON_LIBRARY_NAME})
  ADD_CUSTOM_TARGET(MKDIR_${PYTHON_LIBRARY_NAME}
    mkdir -p dynamic_graph/sot/dynamics/${PYTHON_LIBRARY_NAME}
    )
  SET(NEW_ENTITY_CLASS ${ENTITIES})
  DYNAMIC_GRAPH_PYTHON_MODULE("sot/motion_planner/${PYTHON_LIBRARY_NAME}"
    ${NAME}
    sot/motion_planner/${PYTHON_LIBRARY_NAME}/wrap
    )
ENDFUNCTION()


COMPILE_PLUGIN(localizer localizer.cc Localizer)
COMPILE_PLUGIN(correction correction.cc Correction)
COMPILE_PLUGIN(feet-follower feet-follower.cc FeetFollowerFromFile)


EXEC_PROGRAM(
  ${PYTHON_EXECUTABLE}
  ARGS
  "-c \"from distutils import sysconfig; print sysconfig.get_python_lib(0,0,prefix='')\""
  OUTPUT_VARIABLE PYTHON_SITELIB
  )

INSTALL(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/motion_planner/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/motion_planner
  )
