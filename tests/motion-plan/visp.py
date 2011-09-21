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

import numpy as np

from dynamic_graph.sot.motion_planner.feet_follower import \
    RobotPositionFromVisp

rpfv = RobotPositionFromVisp('rpfv')
rpfv.setSensorTransformation(
    (( 0.,  0., 1., 0.),
     ( 0., -1., 0., 0.),
     (-1.,  0., 0., 0.),
     ( 0.,  0., 0., 1.))
    )

print("local to world:\n" + str(np.matrix(
            (( 0.,  0., 1., 0.),
             ( 0., -1., 0., 0.),
             (-1.,  0., 0., 0.),
             ( 0.,  0., 0., 1.))
            )))

def printDbg():
    print("cMo (local):\n" + str(np.matrix(rpfv.cMo.value)))
    print("cMo (world):\n" + str(np.matrix(rpfv.dbgcMo.value)))
    print("object position (world):\n" + \
              str(np.matrix(rpfv.plannedObjectPosition.value)))
    print("robot position (world):\n" + \
              str(np.matrix(rpfv.dbgPosition.value)))
    print("robot position (XYTheta):\n" + \
              str(rpfv.position.value))

def test(pos, cMo, printDebug = False):
    global rpfv
    rpfv.plannedObjectPosition.value = pos
    rpfv.cMo.value = cMo
    rpfv.cMoTimestamp.value = (42., 42.)

    for s in ['position', 'dbgcMo', 'dbgPosition']:
        rpfv.signal(s).recompute(rpfv.signal(s).time + 1)

    if printDebug:
        printDbg()
    return rpfv.position.value

I = (( 1.,  0., 0., 0.),
     ( 0.,  1., 0., 0.),
     ( 0.,  0., 1., 0.),
     ( 0.,  0., 0., 1.))
X = (( 1.,  0., 0., 1.),
     ( 0.,  1., 0., 0.),
     ( 0.,  0., 1., 0.),
     ( 0.,  0., 0., 1.))
Y = (( 1.,  0., 0., 0.),
     ( 0.,  1., 0., 1.),
     ( 0.,  0., 1., 0.),
     ( 0.,  0., 0., 1.))
Z = (( 1.,  0., 0., 0.),
     ( 0.,  1., 0., 0.),
     ( 0.,  0., 1., 1.),
     ( 0.,  0., 0., 1.))

print("--- I, I")
print(test(I, I) == (0., 0., 0.))
print("--- I, X")
print(test(I, X) == (0., 0., 0.))
print("--- I, Y")
print(test(I, Y) == (0., 1., 0.))
print("--- I, Z")
print(test(I, Z) == (-1., 0., 0.))
