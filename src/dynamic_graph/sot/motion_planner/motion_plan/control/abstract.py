#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of dynamic-graph.
# dynamic-graph is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# dynamic-graph is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

class Control(object):
    def __init__(self, motion, yamlData):
        checkDict('weight', yamlData)
        self.weight = yamlData['weight']

        self.robot = motion.robot

    def start(self, name, feetFollowerWithCorrection):
        raise NotImplementedError

    def interactiveStart(self):
        raise NotImplementedError

    # Configure tracer to store the error estimator entity output.
    # This should be called before exiting the start() method by
    # control elements using the error estimator entity to compute the
    # localization error.
    def setupTraceErrorEstimator(self, errorEstimator):
        for s in ['error',
                  'dbgPositionWorldFrame', 'dbgPlanned', 'dbgIndex',
                  'dbgDeltaCommand', 'dbgDeltaState',
                  'plannedCommand', 'realCommand']:
            addTrace(self.robot, self.trace, errorEstimator.name, s)

    def canStart(self):
        return True
