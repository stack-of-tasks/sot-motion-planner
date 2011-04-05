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
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)

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

SET(FEET_FOLLOWER_SOURCES
  discretized-trajectory.cc
  error-trajectory.cc

  feet-follower.cc
  feet-follower-from-file.cc
  feet-follower-analytical-pg.cc analytical-pg/newPGstepStudy.cpp
  feet-follower-with-correction.cc

  posture-error.cc)

COMPILE_PLUGIN(feet-follower "${FEET_FOLLOWER_SOURCES}" FeetFollowerFromFile)


EXEC_PROGRAM(
  ${PYTHON_EXECUTABLE}
  ARGS
  "-c \"from distutils import sysconfig; print sysconfig.get_python_lib(0,0,prefix='')\""
  OUTPUT_VARIABLE PYTHON_SITELIB
  )



SET(TRAJECTORY_PATH "${CMAKE_SOURCE_DIR}/tests")
CONFIG_FILES(dynamic_graph/sot/motion_planner/feet_follower_graph.py)

SET(PYTHON_MODULE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/motion_planner)
SET(PYTHON_MODULE_BUILD_DIR
  ${CMAKE_CURRENT_BINARY_DIR}/dynamic_graph/sot/motion_planner)

INSTALL(FILES
  ${PYTHON_MODULE_DIR}/__init__.py
  ${PYTHON_MODULE_BUILD_DIR}/feet_follower_graph.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/motion_planner
  )

SET(PYTHON_TEST_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/dynamic_graph/sot/motion_planner/test)
INSTALL(FILES
  ${PYTHON_TEST_DIR}/feet_follower_analytical_pg_walk_on_a_thread.py
  ${PYTHON_TEST_DIR}/feet_follower_analytical_pg_step1.py
  ${PYTHON_TEST_DIR}/feet_follower_analytical_pg_step2.py
  ${PYTHON_TEST_DIR}/feet_follower_from_file_simple.py
  ${PYTHON_TEST_DIR}/feet_follower_from_file_los.py
  ${PYTHON_TEST_DIR}/feet_follower_from_file_walk_on_a_thread.py
  ${PYTHON_TEST_DIR}/feet_follower_analytical_pg_correction.py
  ${PYTHON_TEST_DIR}/__init__.py
  DESTINATION ${PYTHON_SITELIB}/dynamic_graph/sot/motion_planner/test
  )
