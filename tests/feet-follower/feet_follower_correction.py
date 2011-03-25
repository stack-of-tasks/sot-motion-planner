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
import numpy as np
from math import acos, atan2, cos, sin, pi, sqrt

from dynamic_graph.sot.dynamics.tools import *

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import Localizer, FeetFollowerWithCorrection
from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph

from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas


# first slide # hor distance # max feet height # second slide # x # y # theta #
steps = [
    (0.,    0.19, 0.32, -0.62, 0.31,-0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31, 0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31,-0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31, 0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31,-0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31, 0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31,-0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31, 0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.31,-0.19, 0.),
    (-1.95, 0.19, 0.40, -0.62, 0.0,  0.19, 0.),
    ]


f = FeetFollowerAnalyticalPgGraph(steps)

f.referenceTrajectory = f.feetFollower
f.feetFollower = FeetFollowerWithCorrection('correction')

# Set the reference trajectory.
f.feetFollower.setReferenceTrajectory(f.referenceTrajectory.name)

# Set the safety limits.
f.feetFollower.setSafetyLimits(0.01, 0.01, 0.01)

# Make up some error value.
f.feetFollower.offset.value = (0., -0.05, 0.)

# Replug.
plug(f.feetFollower.zmp, robot.device.zmp)
plug(f.feetFollower.com, robot.featureComDes.errorIN)
plug(f.feetFollower.signal('left-ankle'),
     robot.features['left-ankle'].reference)
plug(f.feetFollower.signal('right-ankle'),
     robot.features['right-ankle'].reference)

from common import play; play(f)
