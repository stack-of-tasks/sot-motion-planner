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

        self.type = yamlData['type']

        if self.type != 'feature-point-6d' and self.type != 'feature-com':
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
            if 'rotation' in yamlData and yamlData['rotation']:
                self.selec += '111'
            else:
                self.selec += '000'
            if 'translation' in yamlData and yamlData['translation']:
                self.selec += '111'
            else:
                self.selec += '000'
            motion.robot.features[self.body].selec.value = self.selec

            unlockedDofs = []
            if self.body == 'left-wrist':
                for i in xrange(6):
                    unlockedDofs.append(6 + 12 + 2 + 2 + 7 + i)
            elif self.body == 'right-wrist':
                for i in xrange(6):
                    unlockedDofs.append(6 + 12 + 2 + 2 + i)


            # Push the task into supervisor.
            motion.supervisor.addTask(motion.robot.tasks[self.body].name,
                                      self.interval[0], self.interval[1],
                                      self.priority,
                                      tuple(self.extraUnlockedDofs +
                                            unlockedDofs))

        elif self.type == 'feature-com':
            self.selec = yamlData.get('selec', '111')
            motion.robot.comTask.controlGain.value = self.gain
            motion.robot.featureCom.selec.value = self.selec
            if self.reference == 'static':
                motion.robot.featureComDes.errorIN.value = \
                    motion.robot.dynamic.com.value
            else:
                motion.robot.featureComDes.errorIN.value = \
                    (self.reference.get('x', 0.), self.reference.get('y', 0.))

            # Push the task into supervisor.
            motion.supervisor.addTask(motion.robot.comTask.name,
                                      self.interval[0], self.interval[1],
                                      self.priority,
                                      tuple(self.extraUnlockedDofs))
        else:
            raise RuntimeError('invalid task type')

    def __str__(self):
        return "task motion (type = {0})".format(self.type)

    def setupTrace(self, trace):
        pass

    def canStart(self):
        return True
