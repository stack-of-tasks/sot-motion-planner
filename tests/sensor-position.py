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
# (r, 0) = sensor position
#
# Q = (x, y, theta)
# S = (x, y)
# IM = D
#


l = Localizer('localizer')

r = 1.            # robot radius
error = (1, 0, 0) # dx, dy, dtheta

expectedRobot = (0, 0, 0) # x, y, theta
realRobot = tuple(map(lambda (p, e): p + e, zip(expectedRobot, error))) # x, y, theta

# Sensor positions around the robot:
#
#     1
#   3   2
#     .
#
sensorPos = (0., pi / 2.)

# Features position (x,y)
features = ((10, 10), (200, 4), (-3, -15))

def S((x, y, theta), sensor):
    return (x + r * sin(sensorPos[sensor] + theta), y - r * cos(sensorPos[sensor] + theta))

def P((x, y), l):
    (xr, yr) = features[l]
    return (.5 * (xr - x) * (xr - x) + .5 * (yr - y) * (yr - y),)

def dS((x, y, theta), sensor):
    return ((1, 0, r * cos(sensorPos[sensor] + theta)),
            (0, 1, r * sin(sensorPos[sensor] + theta)))

def dP((x, y), l):
    (xr, yr) = features[l]
    return ((x - xr, y - yr,), )

for sensor in xrange(len(sensorPos)):
    for i in xrange(len(features)):
        prefix = 'obs_{0}_{1}'.format(sensor, i)
        l.add_landmark_observation(prefix)

        l.signal(prefix + '_JfeatureReferencePosition').value = dP(S(expectedRobot, sensor), i)
        l.signal(prefix + '_JsensorPosition').value = dS(expectedRobot, sensor)
        l.signal(prefix + '_weight').value = (1.,)

        l.signal(prefix + '_featureObservedPosition').value =  P(S(realRobot, sensor), i)
        l.signal(prefix + '_featureReferencePosition').value = \
            P(S(expectedRobot, sensor), i)


l.configurationOffset.recompute(0)


for sensor in xrange(len(sensorPos)):
    print "* Sensor {0}:".format(sensor)
    print "\t S({0[0]}, {0[1]}, {0[2]}) = ({1[0]}, {1[1]})".format(
        (0, 0, 0), S((0, 0, 0), sensor))
    print "\t S({0[0]}, {0[1]}, {0[2]}) = ({1[0]}, {1[1]})".format(
        (0, 0, "pi / 2."), S((0, 0, pi / 2.), sensor))
    print ""
    for i in xrange(len(features)):
        print "* Feature value i = {0}:".format(i)
        print "\t P(S({0[0]}, {0[1]}, {0[2]})) = {1[0]}".format(
            (0, 0, 0), P(S((0, 0, 0), sensor), i))
        print "\t P(S({0[0]}, {0[1]}, {0[2]})) = {1[0]}".format(
            (0, 0, "pi / 2."), P(S((0, 0, pi / 2.), sensor), i))
        print ""
    print ""

print "* Result:"

print "\t Initial robot position: " + str(expectedRobot)
for sensor in xrange(len(sensorPos)):
    print "\t Initial sensor {0} position: {1}".format(sensor,  S(expectedRobot, sensor))

offset = l.configurationOffset.value
cfg = tuple(map(lambda (p, e): p + e, zip(expectedRobot, offset)))

print ""
print "\t Real error: " + str(error)
print "\t Real robot position: " + str(realRobot)
for sensor in xrange(len(sensorPos)):
    print "\t Real sensor {0} position: {1}".format(sensor,  S(realRobot, sensor))
print ""
print "\t Correction: " + str(offset)
print "\t Corrected robot position: " + str(cfg)
for sensor in xrange(len(sensorPos)):
    print "\t Corrected sensor {0} position: {1}".format(sensor,  S(cfg, sensor))
print ""

print "* Back projection:"

for sensor in xrange(len(sensorPos)):
    print "* Sensor {0}:".format(sensor)
    for i in xrange(len(features)):
        print "\t Feature " + str(i)
        print "\t - Feature: {0}".format(features[i])
        print "\t - Initial feature pos: {0}".format(P(S(expectedRobot, sensor), i))
        print "\t - Observed feature pos: {0}".format(P(S(realRobot, sensor), i))
        print "\t - Corrected feature pos: {0}".format(P(S(cfg, sensor), i))
        print ""
    print ""
