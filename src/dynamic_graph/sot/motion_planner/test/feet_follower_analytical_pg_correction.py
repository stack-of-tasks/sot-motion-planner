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

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.feet_follower_graph import *
from __main__ import robot, solver

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner \
    import Localizer, FeetFollowerWithCorrection, Randomizer
from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph

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

# Set the safety limits.
#   Checked safety limits.
(maxX, maxY, maxTheta) = (.1, .05, .01)

f.feetFollower.setSafetyLimits(maxX, maxY, maxTheta)
print "Safe limits: %f %f %f" % (maxX, maxY, maxTheta)

# Make up some error value.
r = Randomizer('r')
r.addSignal('offset', 3)
plug (r.offset, f.feetFollower.offset)

# Replug.
plug(f.feetFollower.zmp, robot.device.zmp)
plug(f.feetFollower.com, robot.featureComDes.errorIN)
plug(f.feetFollower.signal('left-ankle'),
     robot.features['left-ankle'].reference)
plug(f.feetFollower.signal('right-ankle'),
     robot.features['right-ankle'].reference)

# Trace
def logRef():
    signals = ['com', 'zmp', 'left-ankle', 'right-ankle']
    for s in signals:
        f.trace.add(f.referenceTrajectory.name + '.' + s,
                    f.referenceTrajectory.name + '-' + s)

        robot.device.after.addSignal(f.referenceTrajectory.name + '.' + s)
        robot.device.after.addSignal(f.feetFollower.name + '.' + s)