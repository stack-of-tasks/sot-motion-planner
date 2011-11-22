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
from dynamic_graph import plug
from dynamic_graph.sot.motion_planner.feet_follower import \
    ErrorMerger, ErrorEstimator

from dynamic_graph.sot.motion_planner.error_estimation_strategy \
    import ErrorEstimationStrategy

from dynamic_graph.sot.motion_planner.motion_plan.tools import *

class MotionPlanErrorEstimationStrategy(ErrorEstimationStrategy):
    errorEstimatorId = 0

    localizationPlannedBody = 'waist'
    errorEstimators = []

    # reference toward the motion plan, added after this object
    # construction and before starting the error estimation.
    motionPlan = None

    def __init__(self, feetFollowerWithCorrection, robot, corba = None):
        ErrorEstimationStrategy.__init__(self,
                                         robot, feetFollowerWithCorrection)
        self.errorEstimator = ErrorMerger(
            '{0}_error_merger'.format(feetFollowerWithCorrection.feetFollower.name))
        self.feetFollowerWithCorrection = feetFollowerWithCorrection
        self.robot = robot

    def start(self, interactive = False):
        for control in self.motionPlan.control:
            name = 'error_estimator' + \
                str(MotionPlanErrorEstimationStrategy.errorEstimatorId)
            MotionPlanErrorEstimationStrategy.errorEstimatorId += 1
            self.errorEstimator.addErrorEstimation(name)

            estimator = None
            if interactive:
                estimator = control.interactiveStart(name,
                                                     self.feetFollowerWithCorrection)
            else:
                estimator = control.start(name, self.feetFollowerWithCorrection)

            if type(estimator) == ErrorEstimator:
                plug(estimator.error,
                     self.errorEstimator.signal("error_" + name))
                self.errorEstimators.append(estimator)
            else:
                # If this is not an error estimator, we suppose it is a constant
                # value that can be used to set the signal.
                self.errorEstimator.signal("error_" + name).value = estimator

            self.errorEstimator.signal("weight_" + name).value = \
                (control.weight,)

            if self.motionPlan.trace and type(estimator) == ErrorEstimator:
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
            for control in self.motionPlan.control:
                control.setupTrace(self.motionPlan.trace)


        return True

    def interactiveStart(self):
        return self.start(interactive = True)

    def __str__(self):
        return "motion plan error estimation strategy"

    def canStart(self):
        return True
