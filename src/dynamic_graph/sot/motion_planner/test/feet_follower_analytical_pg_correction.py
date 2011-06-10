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

from math import cos, sin, atan2
import numpy as np

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
enableCorrection=True
enableOffsetFromErrorEstimator=True

print("* Is this the robot? "  + str(onRobot))
print("* Are we using mocap? " + str(enableMocap))
print("* Is correction enabled? " + str(enableCorrection))
print("* Is correction coming from error estimator (mocap)? " +
      str(enableOffsetFromErrorEstimator))
print("")

# Define correction limits.
#(maxX, maxY, maxTheta) = (0.04, 0.04, 0.1)
#(maxX, maxY, maxTheta) = (0., 0., 0.)

(maxX, maxY, maxTheta) = (0.04, 0., 0.)

# Launch corba server.
corba = CorbaServer('corba')

# first slide # hor distance # max feet height # second slide # x # y # theta
steps= [
    # Front
    (0., 0.24, 0.25, -0.76, 0.15,    -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),

    # To the left.
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),

    # Back
    (0., 0.24, 0.25, -0.76,    -0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, -0.15, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76,  0.,   -0.19, 0.),

    # To the right.
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.25, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.,   -0.19, 0.),
    ]

# Step in place.
steps= [
    (0.,    0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0., -0.19, 0.),
    ]

steps= [
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., -0.19, -0.26),
    (0., 0.24, 0.25, 0., 0., +0.19, -0.26),
    ]

steps= [
    (0., 0.24, 0.25, 0., 0.15, -0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, +0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, -0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, +0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, -0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, +0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, -0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, +0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, -0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, +0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, -0.19, 0.),
    (0., 0.24, 0.25, 0., 0.15, +0.19, 0.),
]


f = FeetFollowerAnalyticalPgGraph(steps)

f.errorEstimator = ErrorEstimator('error_estimator')
f.referenceTrajectory = f.feetFollower
f.corba = corba

if enableCorrection:
    f.feetFollower = FeetFollowerWithCorrection('correction')

    # Set the reference trajectory.
    f.feetFollower.setReferenceTrajectory(f.referenceTrajectory.name)

    plug(robot.dynamic.waist, f.feetFollower.waist)

    # Set the safety limits.
    f.feetFollower.setSafetyLimits(maxX, maxY, maxTheta)
    print ("Safe limits: %f %f %f" % (maxX, maxY, maxTheta))

    # Replug.
    plug(f.feetFollower.zmp, robot.device.zmp)
    plug(f.feetFollower.com, robot.featureComDes.errorIN)
    plug(f.feetFollower.signal('left-ankle'),
         robot.features['left-ankle'].reference)
    plug(f.feetFollower.signal('right-ankle'),
         robot.features['right-ankle'].reference)
    plug(f.feetFollower.signal('waistYaw'),
         robot.features['waist'].reference)

# Setup error estimator.
f.errorEstimator.setReferenceTrajectory(f.referenceTrajectory.name)
plug(robot.dynamic.waist, f.errorEstimator.waist)

if enableCorrection:
    if enableOffsetFromErrorEstimator:
        plug(f.errorEstimator.error, f.feetFollower.offset)
    else:
        # Make up some error value.
        f.randomizer = Randomizer('r')
        f.randomizer.addSignal('offset', 3)
        plug (f.randomizer.offset, f.feetFollower.offset)

def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

def XYThetaToHomogeneousMatrix(x):
    theta = x[2]
    return np.matrix(
        (( cos (theta),-sin (theta), 0., x[0]),
         ( sin (theta), cos (theta), 0., x[1]),
         (          0.,          0., 1., 0.),
         (          0.,          0., 0., 1.))
        )
def HomogeneousMatrixToXYZTheta(x):
    x = np.mat(x)
    return (x[0,3], x[1,3], x[2,3], atan2(x[1,0], x[0,0]))

def computeWorldTransformationFromWaist():
    M = XYThetaToHomogeneousMatrix(corba.waist.value)
    return matrixToTuple(np.matrix(robot.dynamic.waist.value)
                         * np.linalg.inv(M))

# sMm -> position of the mocap frame in the sot frame.
def computeWorldTransformationFromTiles():
    print corba.tiles.value
    theta = corba.tiles.value[2]
    # Tiles position in the mocap frame.
    tMm = XYThetaToHomogeneousMatrix(corba.tiles.value)
    # Tiles position in the SoT frame.
    #sMt = np.matrix(
    #    (( 1., 0., 0., -0.1),
    #     ( 0., 1., 0., -0.1 + 0.19 / 2.),
    #     ( 0., 0., 1., 0.),
    #     ( 0., 0., 0., 1.))
    #    )
    wMm = XYThetaToHomogeneousMatrix(corba.waist.value)
    sMt = wMm * np.linalg.inv(tMm)
    return matrixToTuple(sMt * tMm)

def plugMocap():
    if len(corba.signals()) == 3:
        print ("evart-to-client not launched, abandon.")
        return
    if len(corba.waist.value) != 3:
        print ("waist not tracked, abandon.")
        return
    robot.dynamic.waist.recompute(1)
    #sMm = computeWorldTransformationFromTiles()
    sMm = computeWorldTransformationFromWaist()
    print("World transformation:")
    print(HomogeneousMatrixToXYZTheta(sMm))
    f.errorEstimator.setWorldTransformation(sMm)
    plug(corba.waist, f.errorEstimator.position)
    plug(corba.waistTimestamp, f.errorEstimator.positionTimestamp)
    print ("Initial error:")
    print (f.errorEstimator.error.value)

# Motion capture
if enableMocap:
    if not onRobot:
        while len(corba.signals()) == 3:
            raw_input("Press enter after starting evart-to-corba.")
        while len(corba.waist.value) != 3:
            raw_input("Waist not tracked...")
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
    signals = ['com', 'zmp', 'left-ankle', 'right-ankle', 'waistYaw']
    for s in signals:
        f.trace.add(f.referenceTrajectory.name + '.' + s,
                    f.referenceTrajectory.name + '-' + s)

        robot.device.after.addSignal(f.referenceTrajectory.name + '.' + s)
        robot.device.after.addSignal(f.feetFollower.name + '.' + s)

    if type(f.feetFollower) == FeetFollowerWithCorrection:
        f.trace.add(f.feetFollower.name + '.' + 'offset',
                    f.feetFollower.name + '-' + 'offset')
        robot.device.after.addSignal(f.feetFollower.name + '.' + 'offset')

    f.trace.add(f.errorEstimator.name + '.' + 'error',
                f.errorEstimator.name + '-' + 'error')
    robot.device.after.addSignal(f.errorEstimator.name + '.' + 'error')

    f.trace.add(f.corba.name + '.' + 'waist',
                f.corba.name + '-' + 'waist')
    robot.device.after.addSignal(f.corba.name + '.' + 'waist')

    f.trace.add(robot.device.name + '.' + 'state',
                robot.device.name + '-' + 'state')
    robot.device.after.addSignal(robot.device.name + '.' + 'state')

    f.trace.add(solver.sot.name + '.' + 'control',
                solver.sot.name + '-' + 'control')
    robot.device.after.addSignal(solver.sot.name + '.' + 'control')

    print ("logging reference trajectory")
