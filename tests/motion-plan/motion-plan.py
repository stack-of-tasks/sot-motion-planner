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

import os
import numpy as np
from math import cos, sin, atan2

os.system("rm /tmp/feet_follower_*.dat")

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner import *
from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.robot_viewer import *

logWaist = False
def logWaistTrajectory(robot, f):
    for i in range(4):
        for j in range(4):
            f.write(str(robot.dynamic.waist.value[i][j]) + ' ')
    f.write('\n')


(options, args) = parser.parse_args()

if not len(args):
    raise RuntimeError("motion plan needed")

motionPlan = MotionPlan(args[0], robot, solver)

def play(plan, afterStart = None):
    global logWaist

    elements = []
    if clt:
        elements = clt.listElements()


    maxIter = int(plan.duration / 0.005)
    print maxIter

    while not plan.canStart():
        robot.device.increment(timeStep)
    plan.start()
    print("start")
    if afterStart:
        afterStart()
    print("started")

    t = 0
    startLeft = robot.dynamic.signal('left-ankle').value
    startRight = robot.dynamic.signal('right-ankle').value

    # Main.
    #  Main loop
    #logCfg = open("/tmp/cfg.dat", "w")
    f = open("/tmp/waist.dat", "w")

    if clt:
        if not 'hrp' in elements:
            raise RuntimeError
        drawFootsteps(clt, plan, robot, startLeft, startRight, elements,
                      filename='footsteps-orig.dat')
        drawObstacles(clt, plan, robot, elements)

    for i in xrange(maxIter):
        robot.device.increment(timeStep)

        if clt:
            drawFootsteps(clt, plan, robot, startLeft, startRight,
                          elements, False)
            clt.updateElementConfig(
                'hrp', robot.smallToFull(robot.device.state.value))

        if logWaist:
            logWaistTrajectory(robot, f)


    if plan.feetFollower:
        plan.feetFollower.trace.dump()
    if clt:
        drawFootsteps(clt, plan, robot, startLeft, startRight,
                      elements, False, filename='footsteps-final.dat')

motionPlan.displayMotion()
play(motionPlan)
