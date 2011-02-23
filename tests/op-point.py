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

import robotviewer

import numpy as np

from dynamic_graph import plug
from dynamic_graph.sot.dynamics.hrp2 import Hrp2
from dynamic_graph.sot.core import SOT
from dynamic_graph.sot.core import FeatureGeneric, Task, MatrixConstant

try:
    from dynamic_graph.sot.core import OpPointModifior
    OpPointModifier = OpPointModifior
except ImportError:
    from dynamic_graph.sot.core import OpPointModifier

class Solver:
    robot = None
    sot = None

    def __init__(self, robot):
        self.robot = robot
        self.sot = SOT('solver')
        self.sot.signal('damping').value = 1e-6
        self.sot.setNumberDofs(self.robot.dimension)

        if robot.simu:
            plug(self.sot.signal('control'), robot.simu.signal('control'))
            plug(self.robot.simu.state,
                 self.robot.dynamic.position)


robot = Hrp2("robot")
solver = Solver(robot)
timeStep = 0.005

robot.dynamic.gaze.recompute(0)
robot.dynamic.Jgaze.recompute(0)

lwcam = OpPointModifier('lwcam')
plug(robot.dynamic.gaze, lwcam.positionIN)
plug(robot.dynamic.Jgaze, lwcam.jacobianIN)

# HRP2-14 extrinsic camera parameters
leftWideCamera = ((1., 0., 0., 0.035),
                  (0., 1., 0., 0.072),
                  (0., 0., 1., 0.075),
                  (0., 0., 0., 1.))

lwcam.setTransformation(leftWideCamera)

lwcam.position.recompute(0)
lwcam.jacobian.recompute(0)

print "GAZE"
print robot.dynamic.gaze.value
#print "JGAZE"
#print robot.dynamic.Jgaze.value
print "POSITION"
print lwcam.position.value
#print "JACOBIAN"
print np.asmatrix(lwcam.jacobian.value)[0:6,0:6]


plug(lwcam.position,
     robot.features['gaze'].reference)
solver.sot.push(robot.name + '.task.gaze')

clt = robotviewer.client()
for i in xrange(3000):
    robot.simu.increment(timeStep)
    if clt:
        clt.updateElementConfig(
            'hrp', robot.smallToFull(robot.simu.state.value))
