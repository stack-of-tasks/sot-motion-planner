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

from dynamic_graph.sot.motion_planner import *


(options, args) = parser.parse_args()

if not len(args):
    raise RuntimeError("motion plan needed")

motionPlan = MotionPlan(args[0], robot, solver)

def play(plan, maxIter = 4000, afterStart = None):
    global motionPlan

    while not plan.canStart():
        robot.device.increment(timeStep)
    plan.start()
    print("start")
    if afterStart:
        afterStart()
    print("started")

    t = 0
    # Main.
    #  Main loop
    #logCfg = open("/tmp/cfg.dat", "w")

    for i in xrange(maxIter):
        robot.device.increment(timeStep)

        #log(logCfg)
        if clt:
            clt.updateElementConfig(
                'hrp', robot.smallToFull(robot.device.state.value))

        if plan.feetFollower:
            plan.feetFollower.trace.dump()

motionPlan.displayMotion()
play(motionPlan)
