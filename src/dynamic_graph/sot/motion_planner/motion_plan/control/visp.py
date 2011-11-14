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
from dynamic_graph.sot.core.binary_op import Multiply_matrix_vector
from dynamic_graph.sot.motion_planner.feet_follower import \
    ErrorEstimator
from dynamic_graph.sot.motion_planner.feet_follower import \
    RobotPositionFromVisp
from dynamic_graph.sot.motion_planner.motion_plan.motion import MotionWalk

from dynamic_graph.sot.motion_planner.motion_plan.control.abstract \
    import Control

from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.math import *

from dynamic_graph.ros import *

class ControlViSP(Control):
    yaml_tag = u'visp'

    enableControlFeedback = True

    def __init__(self, motion, yamlData):
        checkDict('object-name', yamlData)
        checkDict('position', yamlData)

        Control.__init__(self, motion, yamlData)

        self.robot = motion.robot

        self.objectName = yamlData['object-name']
        self.frameName = yamlData['frame-name']
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
            self.ros = Ros('ros')

        # Define shorcuts to reduce code verbosity.
        self.rosImport = self.ros.rosImport
        self.rosExport = self.ros.rosExport

        self.rosExport.add('matrixHomoStamped', self.objectName, self.position)

        self.robotPositionFromVisp.plannedObjectPosition.value = \
            obj.plannedPosition.dgRotationMatrix()

        plug(self.rosExport.signal(self.objectName),
             self.robotPositionFromVisp.cMo)
        plug(self.rosExport.signal(self.objectName + 'Timestamp'),
             self.robotPositionFromVisp.cMoTimestamp)

        # Plug wMc/wMr to robotPositionFromVisp
        plug(motion.robot.frames[self.frameName].position,
             self.robotPositionFromVisp.wMc)
        plug(motion.robot.dynamic.waist, self.robotPositionFromVisp.wMr)


        # Compute camera velocity and send it to the tracker.
        # \dot{V_cam} = J_cam * q
        if self.enableControlFeedback:
            if not 'velocityDerivator' in motion.robot.__dict__:
                raise RuntimeError('Control feedback requires velocity computation.')

            self.rosImport.add('twist', 'Vcam', 'Vcam')

            self.VCamEntity = Multiply_matrix_vector(
                'VCamEntity{0}'.format(str(id(yamlData))))

            plug(motion.robot.frames[self.frameName].jacobian,
                 self.VCamEntity.sin1)
            plug(motion.robot.velocityDerivator.sout,
                 self.VCamEntity.sin2)

            plug(self.VCamEntity.sout, self.rosImport.signal('Vcam'))

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
        while len(self.rosExport.signals()) == 0:
            raw_input("Press enter after starting ROS visp_tracker node.")
        while len(self.rosExport.signal(self.objectName).value) < 1:
            raw_input("Tracking not started...")
        return self.start(name, feetFollowerWithCorrection)

    def canStart(self):
        if not self.rosExport:
            return False
        if len(self.rosExport.signals()) == 0:
            return False
        if len(self.rosExport.signal(self.objectName).value) < 1:
            return False
        if len(self.rosExport.signal(self.objectName + 'Timestamp').value) < 1:
            return False
        return True

    def setupTrace(self, errorEstimator):
        self.setupTraceErrorEstimator(self.estimator)
        for s in [self.objectName, self.objectName + 'Timestamp']:
            addTrace(self.robot, self.trace, self.rosExport.name, s)

        for s in ['cMo', 'cMoTimestamp',
                  'plannedObjectPosition',
                  'position',
                  'positionTimestamp',
                  'dbgcMo',
                  'dbgPosition',
                  'dbgrMc']:
            addTrace(self.robot, self.trace, self.robotPositionFromVisp.name, s)

    def __str__(self):
        msg = "ViSP moving edge tracker control element (frame: {0}, object: {1})"
        return msg.format(self.frameName, self.objectName)
