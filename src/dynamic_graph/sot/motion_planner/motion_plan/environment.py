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

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

class EnvironmentObject(object):
    yaml_tag = u'object'

    def __init__(self, motion, yamlData):
        checkDict('planned', yamlData)
        checkDict('estimated', yamlData)

        checkDict('model', yamlData['planned'])
        checkDict('position', yamlData['planned'])
        checkDict('model', yamlData['estimated'])

        self.plannedPosition = Pose6d(yamlData['planned']['position'])
        self.plannedModel = yamlData['planned']['model']
        self.estimatedModel = yamlData['estimated']['model']
