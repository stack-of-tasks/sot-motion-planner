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

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.feet_follower_graph import *
from __main__ import robot, solver

from dynamic_graph.corba_server import CorbaServer

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

def logWaistPos():
    f.trace.add(corba.name + '.waistPosition',
                corba.name + '-waistPosition')
    robot.device.after.addSignal(corba.name + '.waistPosition')

def logWaistPosInteractive():
    print "Press enter after starting evart-to-corba."
    raw_input()
    logWaistPos()
