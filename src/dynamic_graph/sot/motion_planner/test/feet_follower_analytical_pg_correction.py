#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of sot-motion-planner.  sot-motion-planner is free
# software: you can redistribute it and/or modify it under the terms
# of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# sot-motion-planner is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

from dynamic_graph.sot.core import RobotSimu
from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.feet_follower_graph import *
from __main__ import robot, solver

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner \
    import Localizer, FeetFollowerWithCorrection, Randomizer, ErrorEstimator
from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph

from dynamic_graph.corba_server import CorbaServer

onRobot=type(robot.device) != RobotSimu
enableMocap=True

print("Is this the robot? " + str(onRobot))
print("Are we using mocap? "+ str(enableMocap))

# Launch corba server.
corba = CorbaServer('corba')

# first slide # hor distance # max feet height # second slide # x # y # theta
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
f.referenceTrajectory = f.feetFollower
f.feetFollower = FeetFollowerWithCorrection('correction')

# Set the reference trajectory.
f.feetFollower.setReferenceTrajectory(f.referenceTrajectory.name)

plug(robot.dynamic.waist, f.feetFollower.waist)

# Set the safety limits.
# Max rotation alone: 3.14/8.
#(maxX, maxY, maxTheta) = (0.06, 0.03, 0.05)

(maxX, maxY, maxTheta) = (0.04, 0.04, 0.05)

(maxX, maxY, maxTheta) = (0., 0., 0.)

f.feetFollower.setSafetyLimits(maxX, maxY, maxTheta)
print ("Safe limits: %f %f %f" % (maxX, maxY, maxTheta))

# Make up some error value.
f.randomizer = Randomizer('r')
f.randomizer.addSignal('offset', 3)
plug (f.randomizer.offset, f.feetFollower.offset)

f.errorEstimator = ErrorEstimator('error_estimator')
f.errorEstimator.setReferenceTrajectory(f.referenceTrajectory.name)
plug(robot.dynamic.waist, f.errorEstimator.waist)
#plug(f.errorEstimator.error, f.feetFollower.offset)

# Replug.
plug(f.feetFollower.zmp, robot.device.zmp)
plug(f.feetFollower.com, robot.featureComDes.errorIN)
plug(f.feetFollower.signal('left-ankle'),
     robot.features['left-ankle'].reference)
plug(f.feetFollower.signal('right-ankle'),
     robot.features['right-ankle'].reference)

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
        0.
        )
    M = ((1., 0., 0., -x),
         (0., 1., 0., -y),
         (0., 0., 1., -z),
         (0., 0., 0., 1.))
    print (M)

    f.errorEstimator.setWorldTransformation(M)

    plug(corba.waistPosition, f.errorEstimator.position)
    plug(corba.waistPositionTimestamp, f.errorEstimator.positionTimestamp)

# Motion capture
if enableMocap:
    if not onRobot:
        while len(corba.signals()) == 3:
            raw_input("corba")
        plugMocap()
    else:
        print ("Type 'plugMocap()'")
        print ("then 'f.start(logRef)'")
else:
    f.errorEstimator.position.value = (0., 0., 0.)
    f.errorEstimator.positionTimestamp.value = (0., 0.)
    print ("disable error estimator")


# Trace
def logRef():
    signals = ['com', 'zmp', 'left-ankle', 'right-ankle']
    for s in signals:
        f.trace.add(f.referenceTrajectory.name + '.' + s,
                    f.referenceTrajectory.name + '-' + s)

        robot.device.after.addSignal(f.referenceTrajectory.name + '.' + s)
        robot.device.after.addSignal(f.feetFollower.name + '.' + s)

    f.trace.add(robot.dynamic.name + '.' + 'waist',
                robot.dynamic.name + '-' + 'waist')

    f.trace.add(f.feetFollower.name + '.' + 'offset',
                f.feetFollower.name + '-' + 'offset')

    f.trace.add(f.errorEstimator.name + '.' + 'error',
                f.errorEstimator.name + '-' + 'error')
    print ("logging reference trajectory")
