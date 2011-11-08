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

from dynamic_graph import plug
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas
from dynamic_graph.sot.motion_planner.feet_follower import SwayMotionCorrection

robot = Hrp2Laas("robot")
s = SwayMotionCorrection('s')

plug(robot.frames['cameraBottomLeft'].position, s.wMcamera)
plug(robot.dynamic.waist, s.wMwaist)
plug(robot.dynamic.Jcom, s.Jcom)
plug(robot.device.state, s.qdot)

I = ((1., 0., 0., 0.),
     (0., 1., 0., 0.),
     (0., 0., 1., 0.),
     (0., 0., 0., 1.))

s.cMo.value = I
s.cMoTimestamp.value = (0., 0.)
s.initialize()

def recomputeVelocity(cMo = I, inputPgVelocity = (0., 0., 0.)):
    global s
    s.cMo.value = cMo
    s.inputPgVelocity.value = inputPgVelocity
    s.outputPgVelocity.recompute(s.outputPgVelocity.time + 1)
    print("output velocity: {0}".format(s.outputPgVelocity.value))

recomputeVelocity()
