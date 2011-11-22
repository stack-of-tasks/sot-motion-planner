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

class ControlMocap(Control):
    yaml_tag = u'mocap'

    trackedBody = None

    def __init__(self, motion, yamlData):
        checkDict('topic', yamlData)
        checkDict('signal', yamlData)
        Control.__init__(self, motion, yamlData)

        self.robot = motion.robot
        self.topic = yamlData['topic']
        self.signal = yamlData['signal']

        if motion.ros:
            self.ros = motion.ros
        else:
            self.ros = Ros(self.robot)

        # Define shorcuts to reduce code verbosity.
        self.rosImport = self.ros.rosImport
        self.rosExport = self.ros.rosExport

        self.rosExport.add('matrixHomoStamped', self.signal, self.topic)

        self.estimator = ErrorEstimator('estimator{0}'.format(id(yamlData)))
        self.converter = ThreeToTwoDimensionPoseConverter(
            'converter{0}'.format(id(yamlData)))
        plug(self.rosExport.signal(self.signal),
             self.converter.signal('in'))

    def computeWorldTransformationFromFoot(self):
        """
        This methods makes the assumption that the robot is placed
        exactly at its starting position.

        By comparing the current localization with the starting
        position of the tracked body, it deduces the transformation
        between the motion capture system and the control framework.
        """
        self.rosExport.signal(self.signal).recompute(
            self.rosExport.signal(self.signal).time + 1)
        self.robot.dynamic.signal(self.signal).recompute(
            self.robot.dynamic.signal(self.signal).time + 1)

        mocapMfoot = np.matrix(self.rosExport.signal(self.signal).value)
        sotMfoot = np.matrix(self.robot.dynamic.signal(self.signal).value)

        # mocap position w.r.t sot frame
        sotMmocap = sotMfoot * np.linalg.inv(mocapMfoot)
        return matrixToTuple(sotMmocap)


    def start(self, name, feetFollowerWithCorrection):
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)


        # FIXME: we use ankle position as foot position here
        # as Z does not matter.
        plug(feetFollowerWithCorrection.referenceTrajectory.signal(
                self.signal), self.estimator.planned)

        if not self.rosExport:
            return False
        if len(self.rosExport.signals()) == 0:
            return False
        if self.rosExport.signal(self.signal).value[0][0] == 0.:
            print ("tracking failed, abandon. Did you launch evart_bridge?")
            return False
        if self.rosExport.signal(self.signal + 'Timestamp').value[0] == 0.:
            print ("tracking failed (no timestamp), abandon. Did you launch evart_bridge?")
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

        plug(self.converter.out,
             self.estimator.position)
        plug(self.rosExport.signal(self.signal + 'Timestamp'),
             self.estimator.positionTimestamp)
        self.setupTrace(self.estimator)
        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        while not self.rosExport or len(self.rosExport.signals()) == 0:
            raw_input("Press enter after starting evart_bridge.")
        while self.rosExport.signal(self.signal).value[0][0] == 0. or \
                self.rosExport.signal(self.signal + 'Timestamp').value[0] == 0.:
            raw_input("Body not tracked...")
        return self.start(name, feetFollowerWithCorrection)

    def canStart(self):
        if not self.rosExport:
            return False
        if len(self.rosExport.signals()) == 0:
            return False
        if self.rosExport.signal(self.signal).value[0][0] == 0.:
            return False
        if self.rosExport.signal(self.signal + 'Timestamp').value[0] == 0.:
            return False
        return True

    def setupTrace(self, errorEstimator):
        print("fixme")
        self.setupTraceErrorEstimator(self.estimator)
        for s in [self.signal, self.signal + 'Timestamp']:
            addTrace(self.robot, self.trace, self.rosExport.name, s)

        addTrace(self.robot, self.trace, self.converter.name, 'out')

    def __str__(self):
        return "motion capture control element" + \
            " (topic: {0}, signal: {1})".format(self.topic, self.signal)
