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
from dynamic_graph.sot.motion_planner.feet_follower import \
    RobotPositionFromVisp
from dynamic_graph.sot.motion_planner.motion_plan.motion import MotionWalk

from dynamic_graph.sot.motion_planner.motion_plan.control.abstract \
    import Control

from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.ros import RosExport

class ControlViSP(Control):
    yaml_tag = u'visp'

    def __init__(self, motion, yamlData, ros = None):
        checkDict('object-name', yamlData)
        checkDict('position', yamlData)

        Control.__init__(self, motion, yamlData)

        self.robot = motion.robot

        self.objectName = yamlData['object-name']
        self.position = yamlData['position']

        obj = motion.environment.get(self.objectName)
        if not obj:
            raise RuntimeError('object does not exist')

        self.robotPositionFromVisp = RobotPositionFromVisp(
            'robotPositionFromViSP' + str(id(yamlData)))

        if ros:
            self.ros = ros
        else:
            self.ros = RosExport('rosExport')
        self.ros.add('matrixHomoStamped', self.objectName, self.position)

        self.robotPositionFromVisp.plannedObjectPosition.value = \
            obj.plannedPosition.dgRotationMatrix()

        plug(self.ros.signal(self.objectName),
             self.robotPositionFromVisp.cMo)
        plug(self.ros.signal(self.objectName + 'Timestamp'),
             self.robotPositionFromVisp.cMoTimestamp)


    def start(self, name, feetFollowerWithCorrection):
        I = ((1.,0.,0.,0.), (0.,1.,0.,0.), (0.,0.,1.,0.), (0.,0.,0.,1.))
        self.estimator = ErrorEstimator(name)
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)

        # FIXME: gaze is used here, is it correct?
        plug(feetFollowerWithCorrection.referenceTrajectory.gaze,
             self.estimator.planned)

        self.estimator.setSensorToWorldTransformation(I)

        #FIXME: we should change the reference point accordingly
        # with the current contact point.
        plug(self.robot.dynamic.signal('Jleft-ankle'),
             self.estimator.referencePointJacobian)

        self.estimator.plannedCommand.value = self.robot.device.state.value
        if type(self.robot.device) == RobotSimu:
            self.estimator.realCommand.value = self.robot.device.state.value
        else:
            self.estimator.realCommand.value = self.robot.device.robotState.value

        plug(self.robotPositionFromVisp.position, self.estimator.position)

        plug(self.robotPositionFromVisp.positionTimestamp,
             self.estimator.positionTimestamp)

        self.setupTraceErrorEstimator(self.estimator)
        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        return self.start(name, feetFollowerWithCorrection)

    def __str__(self):
        return "ViSP moving edge tracker control element"
