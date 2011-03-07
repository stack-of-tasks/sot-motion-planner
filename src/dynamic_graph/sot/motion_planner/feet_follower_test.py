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

from dynamic_graph.sot.dynamics.tools import *
from __main__ import robot, solver

from dynamic_graph.sot.core import FeatureGeneric, Task, MatrixConstant

from dynamic_graph.sot.motion_planner import FeetFollowerFromFile
feetFollower = FeetFollowerFromFile('feet-follower')


anklePos = robot.dynamic.getAnklePositionInFootFrame()
soleW = robot.dynamic.getSoleWidth()
soleH = robot.dynamic.getSoleLength()
ML = ((1, 0, 0, anklePos[0] + .5 * soleW),
      (0, 1, 0, anklePos[1]),
      (0, 0, 1, anklePos[2]),
      (0, 0, 0, 1))
MR = ((1, 0, 0, anklePos[0] + .5 * soleW),
      (0, 1, 0, -anklePos[1]),
      (0, 0, 1, anklePos[2]),
      (0, 0, 0, 1))
print anklePos

feetFollower.signal('feetToAnkleLeft').value = ML
feetFollower.signal('feetToAnkleRight').value = MR

robot.featureCom.selec.value = '111'

# com start
# 0,0.095,0,0,-0.095,0

#### HALF SITTING #####
taskHalfSit = Task(robot.name + '_halfSit')
featureHS = FeatureGeneric('featureHS')
featureHSdes = FeatureGeneric('featureHSdes')
plug(robot.dynamic.position, featureHS.errorIN)
JHS = MatrixConstant('JHS')
JHS.resize(36, 36)

eye = []
for i in xrange(36):
    tmp = []
    for j in xrange(36):
        if i != j:
            tmp.append(0.)
        else:
            tmp.append(1.)
    eye.append(tuple(tmp))

JHS.set(tuple(eye))

plug(JHS.out, featureHS.jacobianIN)

featureHSdes.errorIN.value = robot.halfSitting
featureHS.sdes.value = featureHSdes


taskHalfSit.add('featureHS')
taskHalfSit.controlGain.value = 180.


plug(feetFollower.signal('com'), robot.featureComDes.errorIN)
plug(feetFollower.signal('left-ankle'),
     robot.features['left-ankle'].reference)
plug(feetFollower.signal('right-ankle'),
     robot.features['right-ankle'].reference)

robot.comTask.signal('controlGain').value = 180.
robot.tasks['left-ankle'].signal('controlGain').value = 180.
robot.tasks['right-ankle'].signal('controlGain').value = 180.

# Push tasks
#  Operational points tasks
solver.sot.push(robot.name + '_task_right-ankle')
solver.sot.push(robot.name + '_task_left-ankle')

#  Center of mass
solver.sot.push(robot.name + '_task_com')

solver.sot.push(robot.name + '_halfSit')
