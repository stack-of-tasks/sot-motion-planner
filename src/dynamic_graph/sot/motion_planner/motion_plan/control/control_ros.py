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
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph.sot.motion_planner.feet_follower import \
    ErrorEstimator, ThreeToTwoDimensionPoseConverter

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.motion.walk import MotionWalk
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.control.abstract import Control

from dynamic_graph.ros import *

class ControlRos(Control):
    yaml_tag = u'control-ros'

    trackedBody = None

    def __init__(self, motion, yamlData):
        checkDict('topic', yamlData)
        checkDict('signal', yamlData)
        Control.__init__(self, motion, yamlData)

        self.motionPlan = motion
        self.topic = yamlData['topic']
        self.signal = yamlData['signal']

        self.motionPlan.ros.rosExport.add('vector3Stamped', self.signal, self.topic)

    def start(self, name, feetFollowerWithCorrection):
        self.estimator = ErrorEstimator(name)
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)

        # FIXME: we use ankle position as foot position here
        # as Z does not matter.
        plug(feetFollowerWithCorrection.referenceTrajectory.signal(
                self.signal), self.estimator.planned)

        if not self.motionPlan.ros.rosExport:
            return False
        if len(self.motionPlan.ros.rosExport.signals()) == 0:
            return False
        if self.motionPlan.ros.rosExport.signal(self.signal).value[0][0] == 0.:
            print ("localization failed, abandon. Did you launch the localization node?")
            return False
        if self.rosExport.signal(self.signal + 'Timestamp').value[0] == 0.:
            print ("localization failed (no timestamp), abandon. "
                   + "Did you launch the localization node?")
            return False

        I = ((1., 0., 0., 0.,),
             (0., 1., 0., 0.,),
             (0., 0., 1., 0.,),
             (0., 0., 0., 1.,),)

        self.estimator.setSensorToWorldTransformation(I)
        plug(self.robot.dynamic.signal('Jwaist'),
             self.estimator.referencePointJacobian)

        self.estimator.plannedCommand.value = self.robot.device.state.value
        if type(self.robot.device) == RobotSimu:
            self.estimator.realCommand.value = self.robot.device.state.value
        else:
            self.estimator.realCommand.value = self.robot.device.robotState.value

        plug(self.motionPlan.ros.rosExport.signal(self.signal),
             self.estimator.position)
        plug(self.motionPlan.ros.rosExport.signal(self.signal + 'Timestamp'),
             self.estimator.positionTimestamp)
        self.setupTrace(self.estimator)
        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        while not self.rosExport or len(self.rosExport.signals()) == 0:
            raw_input("Press enter after starting the localization node.")
        while self.rosExport.signal(self.signal).value[0][0] == 0. or \
                self.rosExport.signal(self.signal + 'Timestamp').value[0] == 0.:
            raw_input("Waiting for localization data...")
        return self.start(name, feetFollowerWithCorrection)

    def canStart(self):
        if not self.motionPlan.ros.rosExport:
            return False
        if len(self.motionPlan.ros.rosExport.signals()) == 0:
            return False
        if self.motionPlan.ros.rosExport.signal(self.signal + 'Timestamp').value[0] == 0.:
            return False
        return True

    def setupTrace(self, errorEstimator):
        self.setupTraceErrorEstimator(self.estimator)
        for s in [self.signal, self.signal + 'Timestamp']:
            self.robot.addTrace(self.rosExport.name, s)

    def __str__(self):
        return "ROS control element" + \
            " (topic: {0}, signal: {1})".format(self.topic, self.signal)
