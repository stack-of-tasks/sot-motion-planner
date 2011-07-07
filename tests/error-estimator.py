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
from time import time

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.feet_follower_graph import *
from dynamic_graph.sot.motion_planner import ErrorEstimator

f = FeetFollowerAnalyticalPgGraph()

e = ErrorEstimator('e')
e.setReferenceTrajectory(f.feetFollower.name)

f.start()

def setup(e, planned, real):
    global i
    robot.device.increment(timeStep)
    e.planned.value = planned
    e.position.value = real
    e.positionTimestamp.value = (time(), 0.,)
    e.error.recompute(e.error.time + 1)
    print e.error.value

def XYThetaToHomogeneousMatrix(xytheta):
    (offsetX, offsetY, offsetTheta) = (xytheta[0], xytheta[1], xytheta[2])
    return ((cos (offsetTheta), -sin(offsetTheta), 0., offsetX),
            (sin (offsetTheta),  cos(offsetTheta), 0., offsetY),
            (               0.,                0., 1.,      0.),
            (               0.,                0., 0.,      1.))

print "null"
setup(e,
      XYThetaToHomogeneousMatrix((0., 0., 0.,)),
      (0., 0., 0.))

print "X translation"
setup(e,
      XYThetaToHomogeneousMatrix((0., 0., 0.,)),
      (10., 0., 0.))

print "Y translation"
setup(e,
      XYThetaToHomogeneousMatrix((0., 0., 0.,)),
      (0., -10., 0.))

print "rotation"
setup(e,
      XYThetaToHomogeneousMatrix((0., 0., 0.5,)),
      (0., 0., 0.))
setup(e,
      XYThetaToHomogeneousMatrix((0., 0., 0.,)),
      (0., 0., 0.5))

print "combined 1"
setup(e,
      XYThetaToHomogeneousMatrix((10., 20., 0.5,)),
      (0., 0., 0.))

print "combined 2"
setup(e,
      XYThetaToHomogeneousMatrix((2., 2., pi / 2.,)),
      (1., 1., 0.))
