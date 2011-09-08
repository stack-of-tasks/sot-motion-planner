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
    ErrorEstimator
from dynamic_graph.sot.motion_planner.feet_follower import VirtualSensor
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

        self.objectName = yamlData['object-name']
        self.position = yamlData['position']

        obj = motion.environment.get(self.objectName)
        if not obj:
            raise RuntimeError('object does not exist')

        self.virtualSensor = VirtualSensor(
            'virtualSensor' + str(id(yamlData)))

        #FIXME: should be more generic.
        feetFollower = find(lambda e: type(e) == MotionWalk, motion.motion)
        if not feetFollower:
            raise RuntimeError('control elements needs at least one ' + \
                                   'walk elelement to apply correction')

        plug(feetFollower.feetFollower.feetFollower.waist,
             self.virtualSensor.expectedRobotPosition)
        plug(motion.robot.dynamic.waist,
             self.virtualSensor.robotPosition)

        if ros:
            self.ros = ros
        else:
            self.ros = RosExport('rosExport')
        self.ros.add('matrixHomoStamped', self.objectName, self.position)

        # FIXME: this is inconsistent.
        # planned is in world frame and localization is relative.
        self.virtualSensor.expectedObstaclePosition.value = \
            obj.plannedPosition.dgRotationMatrix()

        plug(self.ros.signal(self.objectName),
             self.virtualSensor.obstaclePosition)


    def start(self, name, feetFollowerWithCorrection):
        I = ((1.,0.,0.,0.), (0.,1.,0.,0.), (0.,0.,1.,0.), (0.,0.,0.,1.))
        self.estimator = ErrorEstimator(name)
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)

        # FIXME: not generic enough.
        plug(feetFollowerWithCorrection.referenceTrajectory.waist,
             self.estimator.planned)

        self.estimator.setSensorToWorldTransformation(I)

        plug(self.virtualSensor.position, self.estimator.position)
        #FIXME: is it true?
        plug(self.ros.signal(self.objectName + 'Timestamp'),
             self.estimator.positionTimestamp)

        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        return self.start(name, feetFollowerWithCorrection)

    def __str__(self):
        return "ViSP moving edge tracker control element"