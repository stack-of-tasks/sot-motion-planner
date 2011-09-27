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
import numpy as np
from dynamic_graph import plug
from dynamic_graph.sot.core import RobotSimu
from dynamic_graph.sot.core import OpPointModifier
from dynamic_graph.sot.motion_planner.feet_follower import \
    ErrorEstimator
from dynamic_graph.sot.motion_planner.feet_follower import \
    RobotPositionFromVisp
from dynamic_graph.sot.motion_planner.motion_plan.motion import MotionWalk

from dynamic_graph.sot.motion_planner.motion_plan.control.abstract \
    import Control

from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.math import *

from dynamic_graph.ros import RosExport

class ControlViSP(Control):
    yaml_tag = u'visp'

    def __init__(self, motion, yamlData):
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

        # Convert ViSP frame into usual dynamic-graph frame.
        self.robotPositionFromVisp.setSensorTransformation(
            (( 0.,  0., 1., 0.),
             ( 0., -1., 0., 0.),
             (-1.,  0., 0., 0.),
             ( 0.,  0., 0., 1.))
            )

        if motion.ros:
            self.ros = motion.ros
        else:
            self.ros = RosExport('rosExport')
        self.ros.add('matrixHomoStamped', self.objectName, self.position)

        self.robotPositionFromVisp.plannedObjectPosition.value = \
            obj.plannedPosition.dgRotationMatrix()

        plug(self.ros.signal(self.objectName),
             self.robotPositionFromVisp.cMo)
        plug(self.ros.signal(self.objectName + 'Timestamp'),
             self.robotPositionFromVisp.cMoTimestamp)

        # Extrinsic camera parameters.
        g_M_c1 = np.matrix(
            [[1., 0., 0., 0.035],
             [0., 1., 0., 0.072],
             [0., 0., 1., 0.075],
             [0., 0., 0., 1.]])
        g_M_c1 = np.matrix( #FIXME: disabled as long as it has not been checked.
            [[1., 0., 0., 0.],
             [0., 1., 0., 0.],
             [0., 0., 1., 0.],
             [0., 0., 0., 1.]])

        # Frames re-orientation:
        # Z = depth (increase from near to far)
        # X = increase from left to right
        # Y = increase from top to bottom
        c1_M_c = np.matrix(
            [[ 0.,  0.,  1., 0.],
             [-1.,  0.,  0., 0.],
             [ 0., -1.,  0., 0.],
             [ 0.,  0.,  0., 1.]])

        g_M_c = matrixToTuple(g_M_c1 * c1_M_c)

        # Op point modifier.
        self.w_M_c = OpPointModifier('w_M_c'+str(id(yamlData)))
        self.w_M_c.setTransformation(g_M_c)
        plug(motion.robot.dynamic.gaze, self.w_M_c.positionIN)
        plug(motion.robot.dynamic.Jgaze, self.w_M_c.jacobianIN)

        self.w_M_c.position.recompute(self.w_M_c.position.time + 1)
        self.w_M_c.jacobian.recompute(self.w_M_c.jacobian.time + 1)

        # Plug wMc/wMr to robotPositionFromVisp
        plug(self.w_M_c.position, self.robotPositionFromVisp.wMc)
        plug(motion.robot.dynamic.waist, self.robotPositionFromVisp.wMr)



    def start(self, name, feetFollowerWithCorrection):
        I = ((1.,0.,0.,0.), (0.,1.,0.,0.), (0.,0.,1.,0.), (0.,0.,0.,1.))
        self.estimator = ErrorEstimator(name)
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)

        plug(feetFollowerWithCorrection.referenceTrajectory.waist,
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

        self.setupTrace(self.estimator)
        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        while len(self.ros.signals()) == 0:
            raw_input("Press enter after starting ROS visp_tracker node.")
        while len(self.ros.signal(self.objectName).value) < 1:
            raw_input("Tracking not started...")
        return self.start(name, feetFollowerWithCorrection)

    def canStart(self):
        if not self.ros:
            return False
        if len(self.ros.signals()) == 0:
            return False
        if len(self.ros.signal(self.objectName).value) < 1:
            return False
        if len(self.ros.signal(self.objectName + 'Timestamp').value) < 1:
            return False
        return True

    def setupTrace(self, errorEstimator):
        self.setupTraceErrorEstimator(self.estimator)
        for s in [self.objectName, self.objectName + 'Timestamp']:
            addTrace(self.robot, self.trace, self.ros.name, s)

        for s in ['cMo', 'cMoTimestamp',
                  'plannedObjectPosition',
                  'position',
                  'positionTimestamp',
                  'dbgcMo',
                  'dbgPosition']:
            addTrace(self.robot, self.trace, self.robotPositionFromVisp.name, s)

    def __str__(self):
        return "ViSP moving edge tracker control element"
