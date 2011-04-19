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

from dynamic_graph.sot.core import RobotSimu
from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.feet_follower_graph import *
from dynamic_graph.sot.motion_planner \
    import Localizer, FeetFollowerWithCorrection, Randomizer, ErrorEstimator
from __main__ import robot, solver

from dynamic_graph.corba_server import CorbaServer

onRobot=type(robot.device) != RobotSimu
enableMocap=True

print("Is this the robot? " + str(onRobot))
print("Are we using mocap? "+ str(enableMocap))

corba = CorbaServer('corba')

# first slide # hor distance # max feet height # second slide # x # y # theta
steps = [
    (0.,    0.31, 0.15, -0.76, 0.25,-0.19, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.25,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.0,  0.19, 0.),
    ]

steps = [
    (0.,    0.31, 0.15, -0.76, 0.20,-0.19, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20, 0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.20,-0.15, 0.),
    (-1.52, 0.31, 0.15, -0.76, 0.0,  0.19, 0.),
    ]


f = FeetFollowerAnalyticalPgGraph(steps)

f.errorEstimator = ErrorEstimator('error_estimator')
f.errorEstimator.setReferenceTrajectory(f.feetFollower.name)
plug(robot.dynamic.waist, f.errorEstimator.waist)

robot.device.after.addSignal(robot.dynamic.name + '.' + 'waist')

def plugMocap():
    if len(corba.signals()) == 3:
        print ("evart-to-client not launched, abandon.")
        return
    #raw_input("Start evart-to-client, please.")

    #FIXME: here we are supposing that the dg world frame matches
    # the tile frame.
    #f.errorEstimator.setWorldTransformation(corba.tilePosition.value)

    (x, y, z) = (
        corba.waistPosition.value[0],
        corba.waistPosition.value[1],
        robot.dynamic.waist.value[3][3]
        )
    M = ((1., 0., 0., x),
         (0., 1., 0., y),
         (0., 0., 1., z),
         (0., 0., 0., 1.))
    print (M)

    f.errorEstimator.setWorldTransformation(M)

    plug(corba.waistPosition, f.errorEstimator.position)
    plug(corba.waistPositionTimestamp, f.errorEstimator.positionTimestamp)

# Motion capture
if enableMocap:
    if not onRobot:
        while len(corba.signals()) == 3:
            raw_input("Press enter after starting evart-to-corba.")
        plugMocap()
    else:
        print ("Type 'plugMocap()'")
        print ("then 'f.start(logRef)'")
else:
    f.errorEstimator.position.value = (0., 0., 0.)
    f.errorEstimator.positionTimestamp.value = (0., 0.)
    print ("disable error estimator")


def logRef():
    f.trace.add(corba.name + '.waistPosition',
                corba.name + '-waistPosition')
    robot.device.after.addSignal(corba.name + '.waistPosition')

    f.trace.add(f.errorEstimator.name + '.' + 'error',
                f.errorEstimator.name + '-' + 'error')
    robot.device.after.addSignal(f.errorEstimator.name + '.' + 'error')

logWaistPosInteractive = logRef
