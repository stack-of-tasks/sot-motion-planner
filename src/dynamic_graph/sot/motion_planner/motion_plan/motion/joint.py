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
from dynamic_graph.sot.core import FeaturePosture, Task

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.motion.abstract import *

class MotionJoint(Motion):
    yaml_tag = u'joint'

    gain = None
    name = None
    reference = None

    # FIXME: not generic enough and joints are missing.
    nameToId = {
        'left-claw': 35,
        'right-claw': 28
        }

    def __init__(self, motion, yamlData, defaultDirectories):
        checkDict('interval', yamlData)

        Motion.__init__(self, motion, yamlData)

        self.interval = yamlData['interval']
        self.gain = yamlData.get('gain', 1.)
        self.name = yamlData['name']
        self.reference = yamlData['reference']

        self.task = Task('joint'+str(id(yamlData)))
        self.feature = FeaturePosture('jointFeaturePosture'+str(id(yamlData)))
        plug(motion.robot.device.state, self.feature.state)

        jointId = self.nameToId[self.name]

        posture = np.array(motion.robot.halfSitting)
        posture[jointId] = self.reference

        self.feature.setPosture(tuple(posture.tolist()))

        for i in xrange(len(posture) - 6):
            if i + 6 == jointId:
                self.feature.selectDof(i + 6, True)
            else:
                self.feature.selectDof(i + 6, False)

        self.task.add(self.feature.name)
        self.task.controlGain.value = self.gain

        # Push the task into supervisor.
        motion.supervisor.addTask(self.task.name,
                                  self.interval[0], self.interval[1])

    def __str__(self):
        return "joint motion ({0})".format(self.name)

    def setupTrace(self, trace):
        pass
