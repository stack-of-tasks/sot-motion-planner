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

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import Localizer


l = Localizer('localizer')
l.add_landmark_observation('obs')

# Feature space is geometrical space without the orientation.
# Here we would have some kind of "global" detector
# returning (x,y) world coordinate of some objects.
l.obs_JfeatureReferencePosition.value = (
    (1, 0, 0),
    (0, 1, 0)
    )

# Robot is sensor, so jacobian is identity.
l.obs_JsensorPosition.value = (
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    )

l.obs_featureObservedPosition.value = (0, 0)
l.obs_featureReferencePosition.value = (0, 0)

# Equal weight for X/Y of the only one observation.
l.obs_weight.value = (1., 1.)

l.configurationOffset.recompute(0)
print l.configurationOffset.value
