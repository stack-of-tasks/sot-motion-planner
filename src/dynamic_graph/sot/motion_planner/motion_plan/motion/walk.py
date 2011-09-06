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
from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.motion.abstract import *

class MotionWalk(Motion):
    yaml_tag = u'walk'

    footsteps = None
    waistFile = None
    feetFollower = None

    def __init__(self, motion, yamlData):
        checkDict('footsteps', yamlData)

        steps = convertToNPFootstepsStack(yamlData['footsteps'])
        self.footsteps = yamlData['footsteps']

        #FIXME: handle multiple walk movement.
        motion.footsteps = yamlData['footsteps']

        self.waistFile = yamlData.get('waist-trajectory')
        self.feetFollower = FeetFollowerAnalyticalPgGraph(
            motion.robot, motion.solver, steps,
            waistFile = self.waistFile)
        #FIXME: ...
        motion.trace = self.feetFollower.trace

    def __str__(self):
        return "walking motion ({0} footstep(s))".format(len(self.footsteps))
