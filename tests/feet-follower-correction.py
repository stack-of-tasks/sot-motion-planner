#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of sot-motion-planner.
# sot-motion-planner is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-motion-planner is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

from __future__ import print_function
import numpy as np
from math import acos, atan2, cos, sin, pi, sqrt

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import \
    Localizer, Correction, FeetFollowerFromFile

from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas

try:
    from dynamic_graph.sot.core import OpPointModifior
    OpPointModifier = OpPointModifior
except ImportError:
    from dynamic_graph.sot.core import OpPointModifier

from dynamic_graph.sot.core import FeatureVisualPoint

def makePosition(tx, ty, tz):
    return ((1., 0., 0., tx),
            (0., 1., 0., ty),
            (0., 0., 1., tz),
            (0., 0., 0., 1.))


###############################################################
# Feet follower setup

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
feetFollower.signal('feetToAnkleLeft').value = ML
feetFollower.signal('feetToAnkleRight').value = MR
robot.featureCom.selec.value = '111'


###############################################################
# Half sitting task

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


###############################################################
# Correction

correction = Correction('correction')

# Plug feet follower into correction
plug(feetFollower.com, correction.trajectoryComIn)
plug(feetFollower.signal('left-ankle'),
     correction.trajectoryLeftFootIn)
plug(feetFollower.signal('right-ankle'),
     correction.trajectoryRightFootIn)

# Plug correction into features sdes.
plug(correction.trajectoryCom, robot.featureComDes.errorIN)
plug(correction.trajectoryLeftFoot,
     robot.features['left-ankle'].reference)
plug(correction.trajectoryRightFoot,
     robot.features['right-ankle'].reference)


###############################################################
# Tasks

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


# Setup the initial error.
correction.offset.value = (1., 2., 0.)

for i in xrange(2 * (1. / 0.005)):
    robot.device.increment(timeStep)
    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.device.state.value))

    # Nullify the error after (at least) one step.
    correction.offset.value = (0., 0., 0)
