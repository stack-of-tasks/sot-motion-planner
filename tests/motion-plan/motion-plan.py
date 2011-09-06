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

from __future__ import print_function
import yaml

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.motion_plan import *
from dynamic_graph.sot.motion_planner.motion_plan.viewer import *

(options, args) = parser.parse_args()

if not len(args):
    raise RuntimeError("motion plan needed")

try:
    motionPlan = MotionPlan(args[0], robot, solver)
    print(motionPlan)
    if clt:
        motionPlanViewer = MotionPlanViewer(motionPlan, robot, clt,
                                            motionPlan.logger,
                                            logOpPoints = False)
        motionPlanViewer.play()
except yaml.YAMLError, e:
    print("failed to parse YAML file: " + str(e))
