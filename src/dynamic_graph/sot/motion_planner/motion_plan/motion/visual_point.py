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
from dynamic_graph.sot.core import FeatureVisualPoint, Task
from dynamic_graph.sot.motion_planner import VispPointProjection
from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.motion.abstract import *

class MotionVisualPoint(Motion):
    yaml_tag = u'visual-point'

    type = None
    gain = None
    objectName = None

    def __init__(self, motion, yamlData, defaultDirectories):
        checkDict('interval', yamlData)

        Motion.__init__(self, motion, yamlData)

        self.objectName = yamlData['object-name']
        self.frameName = yamlData['frame-name']

        self.gain = yamlData.get('gain', 1.)

        # Cannot change the stack dynamically for now.
        if self.interval[0] != 0 and self.interval[1] != motion.duration:
            raise NotImplementedError

        # Desired feature
        self.fvpDes = FeatureVisualPoint('fvpDes'+str(id(yamlData)))
        self.fvpDes.xy.value = (0., 0.)

        # Feature
        self.vispPointProjection = VispPointProjection('vpp'+str(id(yamlData)))
        self.vispPointProjection.cMo.value = (
            (1., 0., 0., 0.),
            (0., 1., 0., 0.),
            (0., 0., 1., 1.),
            (0., 0., 0., 1.),)
        self.vispPointProjection.cMoTimestamp.value = (0., 0.)

        self.fvp = FeatureVisualPoint('fvp'+str(id(yamlData)))
        plug(self.vispPointProjection.xy, self.fvp.xy)
        plug(self.vispPointProjection.Z, self.fvp.Z)

        self.fvp.Z.value = 1.
        self.fvp.sdes.value = self.fvpDes
        self.fvp.selec.value = '11'
        plug(motion.robot.frames[self.frameName].jacobian, self.fvp.Jq)

        self.fvp.error.recompute(self.fvp.error.time + 1)
        self.fvp.jacobian.recompute(self.fvp.jacobian.time + 1)

        # Task
        self.task = Task('task_fvp_'+str(id(yamlData)))
        self.task.add(self.fvp.name)
        self.task.controlGain.value = self.gain

        self.task.error.recompute(self.task.error.time + 1)
        self.task.jacobian.recompute(self.task.jacobian.time + 1)

        # Push the task into supervisor.
        motion.supervisor.addTask(self.task.name,
                                  self.interval[0], self.interval[1],
                                  self.priority,
                                  #FIXME: HRP-2 specific
                                  tuple(self.extraUnlockedDofs) +
                                  (6 + 14, 6 + 15))

    def __str__(self):
        msg = "visual point motion (frame: {0}, object: {1})"
        return msg.format(self.frameName, self.objectName)

    def setupTrace(self, trace):
        for s in ['xy', 'Z']:
            addTrace(self.robot, trace, self.vispPointProjection.name, s)

        for s in ['xy']:
            addTrace(self.robot, trace, self.fvpDes.name, s)

        for s in ['xy', 'Z', 'error']:
            addTrace(self.robot, trace, self.fvp.name, s)

        for s in ['error']:
            addTrace(self.robot, trace, self.task.name, s)
