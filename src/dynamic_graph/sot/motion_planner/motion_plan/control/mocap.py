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
    ErrorEstimator

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.motion.walk import MotionWalk
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.control.abstract import Control

class ControlMocap(Control):
    yaml_tag = u'mocap'

    trackedBody = None

    def __init__(self, motion, yamlData):
        checkDict('tracked-body', yamlData)
        checkDict('perceived-body', yamlData)
        Control.__init__(self, motion, yamlData)

        self.corba = motion.corba
        self.robot = motion.robot
        self.trackedBody = yamlData['tracked-body']
        self.perceivedBody = yamlData['perceived-body']

    def computeWorldTransformationFromFoot(self):
        """
        This methods makes the assumption that the robot is placed
        exactly at its starting position.

        By comparing the current localization with the starting
        position of the tracked body, it deduces the transformation
        between the motion capture system and the control framework.
        """
        self.corba.signal(self.perceivedBody).recompute(
            self.corba.signal(self.perceivedBody).time + 1)
        self.robot.dynamic.signal(
            self.trackedBody).recompute(self.robot.dynamic.signal(
                self.trackedBody).time + 1)

        mocapMfoot = XYThetaToHomogeneousMatrix(
            self.corba.signal(self.perceivedBody).value)
        sotMfoot = np.matrix(self.robot.dynamic.signal(
                self.trackedBody).value)

        # mocap position w.r.t sot frame
        sotMmocap = sotMfoot * np.linalg.inv(mocapMfoot)
        return matrixToTuple(sotMmocap)


    def start(self, name, feetFollowerWithCorrection):
        self.estimator = ErrorEstimator(name)
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)


        # FIXME: we use ankle position as foot position here
        # as Z does not matter.
        plug(feetFollowerWithCorrection.referenceTrajectory.signal(
                self.trackedBody), self.estimator.planned)

        if len(self.corba.signals()) == 3:
            print ("evart-to-client not launched, abandon.")
            return False
        if len(self.corba.signal(self.perceivedBody).value) != 3:
            print ("{0} not tracked, abandon.".format(self.perceivedBody))
            return False

        sMm = self.computeWorldTransformationFromFoot()

        self.estimator.setSensorToWorldTransformation(sMm)

        #FIXME: we should change the reference point accordingly
        # with the current contact point.
        plug(self.robot.dynamic.signal('Jleft-ankle'),
             self.estimator.referencePointJacobian)

        self.estimator.plannedCommand.value = self.robot.device.state.value
        if type(self.robot.device) == RobotSimu:
            self.estimator.realCommand.value = self.robot.device.state.value
        else:
            self.estimator.realCommand.value = self.robot.device.robotState.value

        plug(self.corba.signal(self.perceivedBody),
             self.estimator.position)
        plug(self.corba.signal(
                self.perceivedBody + 'Timestamp'),
             self.estimator.positionTimestamp)
        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        while len(self.corba.signals()) == 3:
            raw_input("Press enter after starting evart-to-corba.")
        while len(self.corba.signal(self.perceivedBody).value) != 3:
            raw_input("Body not tracked...")
        return self.start(name, feetFollowerWithCorrection)

    def canStart(self):
        if not self.corba:
            return False
        if len(self.corba.signals()) == 3:
            return False
        if len(self.corba.signal(self.perceivedBody).value) != 3:
            return False
        return True

    def __str__(self):
        return "motion capture control element" + \
            " (tracked: {0}, perceived: {1})".format(
            self.trackedBody, self.perceivedBody)
