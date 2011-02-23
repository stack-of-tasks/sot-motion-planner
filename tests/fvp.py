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
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas

try:
    from dynamic_graph.sot.core import OpPointModifior
    OpPointModifier = OpPointModifior
except ImportError:
    from dynamic_graph.sot.core import OpPointModifier

from dynamic_graph.sot.core import FeatureVisualPoint, Task

from dynamic_graph.sot.dynamics.tools import *

def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)



robot.dynamic.gaze.recompute(0)
robot.dynamic.Jgaze.recompute(0)

# Camera related frames.
w_M_c = OpPointModifier('w_M_c')

# Compute the transformation between the
# gaze and the left bottom (wide) camera.

# Extrinsic camera parameters.
g_M_c1 = np.matrix(
    [[1., 0., 0., -0.072],
     [0., 1., 0., -0.075],
     [0., 0., 1.,  0.035],
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

plug(robot.dynamic.gaze, w_M_c.positionIN)
plug(robot.dynamic.Jgaze, w_M_c.jacobianIN)

w_M_c.setTransformation(g_M_c)
w_M_c.position.recompute(0)
w_M_c.jacobian.recompute(0)

def Xw(t):
    robot.dynamic.signal('right-wrist').recompute(t)
    x = np.matrix(robot.dynamic.signal('right-wrist').value)[0,3]
    y = np.matrix(robot.dynamic.signal('right-wrist').value)[1,3]
    z = np.matrix(robot.dynamic.signal('right-wrist').value)[2,3]
    Xw = np.array([x, y, z, 1.])
    return Xw


f = 1.
(px, py) = (1., 1.)
(u0, v0) = (0., 0.)

def S(q, sensorId):
    return np.matrix(w_M_c.position.value, dtype=np.float)

def split(P):
    return (P[0], P[1], P[2])
def P(sensorPosition, referencePoint):
    referencePoint_ = np.inner(np.linalg.inv(sensorPosition),
                               referencePoint)
    (X, Y, Z) = split(referencePoint_)

    x = u0 + f * px * X / Z
    y = v0 + f * py * Y / Z
    return np.array([x, y], dtype=np.float)


# Make sure feet and wrists don't move.
robot.features['left-ankle'].reference.value = \
    robot.dynamic.signal('left-ankle').value
robot.features['left-ankle']._feature.signal('selec').value = '111111'
robot.features['right-ankle'].reference.value = \
    robot.dynamic.signal('right-ankle').value
robot.features['right-ankle']._feature.signal('selec').value = '111111'
robot.features['left-wrist'].reference.value = \
    robot.dynamic.signal('left-wrist').value
robot.features['left-wrist']._feature.signal('selec').value = '111111'
robot.features['right-wrist'].reference.value = \
    robot.dynamic.signal('right-wrist').value
robot.features['right-wrist']._feature.signal('selec').value = '111111'


robot.tasks['left-ankle'].controlGain.value = 1.
robot.tasks['right-ankle'].controlGain.value = 1.
robot.tasks['left-wrist'].controlGain.value = 1.
robot.tasks['right-wrist'].controlGain.value = 1.

solver.sot.push(robot.name + '_task_left-ankle')
solver.sot.push(robot.name + '_task_right-ankle')
solver.sot.push(robot.name + '_task_left-wrist')
solver.sot.push(robot.name + '_task_right-wrist')


# Feature visual point
fvp = FeatureVisualPoint('fvp')
fvp.xy.value = (P(S(0,0), Xw(0))[0], P(S(0,0), Xw(0))[1])
fvp.Z.value = 1. + np.inner(np.linalg.inv(S(0,0)), Xw(0))[2]
plug(w_M_c.jacobian, fvp.Jq)

fvp_sdes = FeatureVisualPoint('fvp_sdes')
fvp_sdes.xy.value = (0., 0.)

fvp.sdes.value = fvp_sdes


task = Task('fvp_task')
task.add('fvp')
task.controlGain.value = 10.
solver.sot.push('fvp_task')


t = 0
for i in xrange(0,2000):
    robot.device.increment(timeStep)

    w_M_c.position.recompute(t)
    w_M_c.jacobian.recompute(t)

    fvp.xy.value = (P(S(0,0), Xw(t))[0], P(S(0,0), Xw(t))[1])

    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.device.state.value))

    t += 1
