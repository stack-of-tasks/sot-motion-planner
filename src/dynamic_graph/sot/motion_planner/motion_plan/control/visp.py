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
    ErrorEstimator

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

        if ros:
            self.ros = ros
        else:
            self.ros = RosExport('rosExport')
        self.ros.add('matrixHomoStamped', self.objectName, self.position)

    def start(self, name, feetFollowerWithCorrection):
        I = ((1.,0.,0.,0.), (0.,1.,0.,0.), (0.,0.,1.,0.), (0.,0.,0.,1.))
        self.estimator = ErrorEstimator(name)
        self.estimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)

        # FIXME: not generic enough.
        plug(feetFollowerWithCorrection.referenceTrajectory.waist,
             self.estimator.planned)

        self.estimator.setSensorToWorldTransformation(I)

        #FIXME: write ViSP error estimation entity.
        #plug(self.ros.signal(self.objectName), self.estimator.position)
        self.estimator.position.value = (0., 0., 0.)
        plug(self.ros.signal(self.objectName + 'Timestamp'),
             self.estimator.positionTimestamp)

        return self.estimator

    def interactiveStart(self, name, feetFollowerWithCorrection):
        return self.start(name, feetFollowerWithCorrection)

    def __str__(self):
        return "ViSP moving edge tracker control element"
