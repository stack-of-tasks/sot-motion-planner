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
    import Localizer, FeetFollowerWithCorrection
from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph

# first slide # hor distance # max feet height # second slide # x # y # theta
steps = [
    (0.,    0.19, 0.10, -0.62, 0.0,-0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0, 0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0,-0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0, 0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0,-0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0, 0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0,-0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0, 0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0,-0.19, 0.),
    (-1.95, 0.19, 0.10, -0.62, 0.0,  0.19, 0.),
    ]


f = FeetFollowerAnalyticalPgGraph(steps)
f.referenceTrajectory = f.feetFollower
f.feetFollower = FeetFollowerWithCorrection('correction')

# Set the reference trajectory.
f.feetFollower.setReferenceTrajectory(f.referenceTrajectory.name)

# Set the safety limits.
maxX = robot.dynamic.getSoleLength () / 4.
maxY = robot.dynamic.getSoleWidth () / 4.
maxTheta = 0.01
f.feetFollower.setSafetyLimits(maxX, maxY, maxTheta)
print "Safe limits: %f %f %f" % (maxX, maxY, maxTheta)

# Make up some error value.
offset = (maxX, maxY, 0.)
f.feetFollower.offset.value = offset
print "Offset: %f %f %f" % offset

# Replug.
plug(f.feetFollower.zmp, robot.device.zmp)
plug(f.feetFollower.com, robot.featureComDes.errorIN)
plug(f.feetFollower.signal('left-ankle'),
     robot.features['left-ankle'].reference)
plug(f.feetFollower.signal('right-ankle'),
     robot.features['right-ankle'].reference)

# Trace
def logRef():
    f.trace.add(f.referenceTrajectory.name + '.com',
                f.referenceTrajectory.name + '-com')
    f.trace.add(f.referenceTrajectory.name + '.zmp',
                f.referenceTrajectory.name + '-zmp')
    f.trace.add(f.referenceTrajectory.name + '.left-ankle',
                f.referenceTrajectory.name + '-left-ankle')
    f.trace.add(f.referenceTrajectory.name + '.right-ankle',
                f.referenceTrajectory.name + '-right-ankle')
