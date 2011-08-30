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

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner.feet_follower import \
    ErrorMerger

from dynamic_graph.sot.motion_planner.error_estimation_strategy \
    import ErrorEstimationStrategy

from dynamic_graph.sot.motion_planner.motion_plan.tools import *

class MotionPlanErrorEstimationStrategy(ErrorEstimationStrategy):
    localizationPlannedBody = 'waist'
    errorEstimators = []

    # reference toward the motion plan, added after this object
    # construction and before starting the error estimation.
    motionPlan = None

    def __init__(self, feetFollowerWithCorrection, robot, corba = None):
        ErrorEstimationStrategy.__init__(self,
                                         robot, feetFollowerWithCorrection)
        self.errorEstimator = ErrorMerger('error_merger')
        self.feetFollowerWithCorrection = feetFollowerWithCorrection
        self.robot = robot

    def start(self, interactive = False):
        for control in self.motionPlan.control:
            name = 'error_estimator' + str(id(control))
            self.errorEstimator.addErrorEstimation(name)

            estimator = None
            if interactive:
                estimator = control.interactiveStart(name,
                                                     self.feetFollowerWithCorrection)
            else:
                estimator = control.start(name, self.feetFollowerWithCorrection)
            plug(estimator.error,
                 self.errorEstimator.signal("error_" + name))
            self.errorEstimator.signal("weight_" + name).value = \
                (control.weight,)
            self.errorEstimators.append(estimator)

            if self.motionPlan.trace:
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'error')
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'dbgPositionWorldFrame')
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'dbgPlanned')
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'dbgIndex')

        if self.motionPlan.trace:
            addTrace(self.motionPlan.robot,
                     self.motionPlan.trace,
                     self.errorEstimator.name, 'error')
        return True

    def interactiveStart(self):
        return self.start(interactive = True)

    def __str__(self):
        return "motion plan error estimation strategy"
