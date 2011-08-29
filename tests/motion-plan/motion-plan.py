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
import logging
import yaml

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner import *

(options, args) = parser.parse_args()

if not len(args):
    raise RuntimeError("motion plan needed")

def initializeLogging():
    logger = logging.getLogger('motion-plan')
    logger.setLevel(logging.INFO)

    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    # create formatter
    fmt = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    formatter = logging.Formatter(fmt)

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)
    return logger

try:
    logger = initializeLogging()
    motionPlan = MotionPlan(args[0], robot, solver)
    if clt:
        motionPlanViewer = MotionPlanViewer(motionPlan, robot, clt, logger)
        motionPlanViewer.play()
except yaml.YAMLError, e:
    print("failed to parse YAML file: " + str(e))
