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
from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.motion.abstract import *

class MotionTask(Motion):
    yaml_tag = u'task'

    type = None
    gain = None
    reference = None

    def __init__(self, motion, yamlData, defaultDirectories):
        checkDict('interval', yamlData)

        Motion.__init__(self, motion, yamlData)

        if len(yamlData['interval']) != 2:
            raise RuntimeErrror('invalid interval')

        self.interval = yamlData['interval']
        self.type = yamlData['type']

        if self.type != 'feature-point-6d' and self.type != 'feature-com':
            raise NotImplementedError
        # Cannot change the stack dynamically for now.
        if self.interval[0] != 0 and self.interval[1] != motion.duration:
            raise NotImplementedError

        self.gain = yamlData['gain']
        self.reference = yamlData['reference']

        if self.type == 'feature-point-6d':
            self.body = yamlData['operational-point']

            motion.robot.tasks[self.body].controlGain.value = self.gain
            if self.reference == 'static':
                motion.robot.features[self.body]._reference.position.value = \
                    motion.robot.dynamic.signal(self.body).value
            else:
                p = Pose6d(self.reference)
                motion.robot.features[self.body]._reference.position.value = \
                    p.dgRotationMatrix()

            self.selec = ''
            if 'translation' in yamlData and yamlData['translation']:
                self.selec += '111'
            else:
                self.selec += '000'
            if 'rotation' in yamlData and yamlData['rotation']:
                self.selec += '111'
            else:
                self.selec += '000'
            motion.robot.features[self.body].selec.value = self.selec
            motion.solver.sot.push(motion.robot.tasks[self.body].name)
        elif self.type == 'feature-com':
            motion.robot.comTask.controlGain.value = self.gain
            if self.reference == 'static':
                motion.robot.featureComDes.errorIN.value = \
                    motion.robot.dynamic.com.value
            else:
                motion.robot.featureComDes.position.value = \
                    (self.reference.get('x', 0.), self.reference.get('y', 0.))
            motion.solver.sot.push(motion.robot.comTask.name)
        else:
            raise RuntimeError('invalid task type')

    def __str__(self):
        return "task motion (type = {0})".format(self.type)

    def setupTrace(self, trace):
        pass
