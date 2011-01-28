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

from math import cos, sin, pi

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import Localizer

#
#
#       X featurePos
#
#
#       . sensor
#       O robot (circle, radius = r)
#
#
#        .
#       / \
#        | x+
#  y+ <--X
#
# (0, 0) = O = robot position


l = Localizer('localizer')
l.add_landmark_observation('obs')

r = 1.
robot = (0, 0, 0)
error = (1, 0, 0)

featurePos = (10, 10)

def computeObservedFeaturePos((x, y, theta), (ex, ey, etheta)):
    """Compute the observed feature position
    from a position error."""

    (rsx, rsy) = S((x + ex, y + ey, theta + etheta))
    (fx, fy) = featurePos

    return (fx - rsx, fy - rsy)

def S((x, y, theta)):
    return (x + r * sin(theta), y - r * cos(theta))

def P((x, y)):
    (xr, yr) = featurePos
    return (.5 * (xr - x) * (xr - x) + .5 * (yr - y) * (yr - y),)

def dS((x, y, theta)):
    return ((1, 0, r * cos(theta)),
            (0, 1, r * sin(theta)))

def dP((x, y)):
    (xr, yr) = featurePos
    return ((x - xr, y - yr,), )

observedFeaturePos = computeObservedFeaturePos(robot, error)



# First observation.
l.obs_JfeatureReferencePosition.value = dP(S(robot))
l.obs_JsensorPosition.value = dS(robot)
l.obs_weight.value = (1.,)

l.obs_featureObservedPosition.value =  P(featurePos)
l.obs_featureReferencePosition.value = P(observedFeaturePos)

l.configurationOffset.recompute(0)
print l.configurationOffset.value
