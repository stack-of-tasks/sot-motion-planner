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

from dynamic_graph.sot.motion_planner.motion_plan.tools import *
from dynamic_graph.sot.motion_planner.motion_plan.control.abstract import Control


class ControlConstant(Control):
    yaml_tag = u'constant'

    def __init__(self, motion, yamlData):
        checkDict('error', yamlData)

        Control.__init__(self, motion, yamlData)
        self.error = (yamlData['error']['x'],
                      yamlData['error']['y'],
                      yamlData['error']['theta'])

    def start(self, name, feetFollowerWithCorrection):
        return self.error

    def interactiveStart(self, name, feetFollowerWithCorrection):
        return self.start(name, feetFollowerWithCorrection)

    def __str__(self):
        return "constant error control element" + \
            " (error: {0})".format(self.error)
