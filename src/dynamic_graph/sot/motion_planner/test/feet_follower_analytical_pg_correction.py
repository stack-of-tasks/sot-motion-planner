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

from __future__ import print_function
from __main__ import robot, solver

from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph
from dynamic_graph.sot.motion_planner.feet_follower_graph_with_correction \
    import FeetFollowerGraphWithCorrection
from dynamic_graph.sot.motion_planner.error_estimation_strategy \
    import MotionCaptureErrorEstimationStrategy

steps = [
    ( 0.000000, 0.24, 0.25, -0.760000, 0.100000, -0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.050000, 0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, -0.170000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, 0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, -0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, -0.170000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, -0.050000, 0.170000, 0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, -0.170000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, -0.170000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, 0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, -0.190000, 0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.190000, 0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, -0.000000, -0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, -0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.190000, 0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, -0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.190000, 0.000000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, -0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.150000, 0.170000, 0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, -0.190000, -0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.050000, 0.190000, 0.130000,),
    (-1.520000, 0.24, 0.25, -0.760000, 0.100000, -0.170000, 0.000000),
    ]

steps= [
    (+0.,    0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
    (-1.52, 0.24, 0.25, -0.76, 0.1, -0.19, 0.),
    (-1.52,  0.24, 0.25, -0.76, 0.1, +0.19, 0.),
]



feetFollower = FeetFollowerAnalyticalPgGraph(robot, solver, steps)
correctedFeetFollower = FeetFollowerGraphWithCorrection(
    robot, solver, feetFollower, MotionCaptureErrorEstimationStrategy,
    maxX=0., maxY=0., maxTheta=0.)

# Short alias.
f = correctedFeetFollower

print(correctedFeetFollower)

# If not on robot, start automatically.
if not correctedFeetFollower.onRobot:
    f.start()
