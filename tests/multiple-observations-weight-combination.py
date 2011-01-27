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
l.add_landmark_observation('obs2')

# Feature space is geometrical space without the orientation.
# Here we would have some kind of "global" detector
# returning (x,y) world coordinate of some objects.
JfeatureReferencePosition = (
    (1, 0, 0),
    (0, 1, 0)
    )

# Robot is sensor, so jacobian is identity.
JsensorPosition = (
    (1, 0, 0),
    (0, 1, 0),
    (0, 0, 1),
    )


# First observation.
l.obs_JfeatureReferencePosition.value = JfeatureReferencePosition
l.obs_JsensorPosition.value = JsensorPosition
l.obs_sensorPosition.value = (0, 0, 0)
l.obs_weight.value = (0., 1.)

l.obs_featureObservedPosition.value = (99999, 0)
l.obs_featureReferencePosition.value = (0, 3)

# Second observation.
l.obs2_JfeatureReferencePosition.value = JfeatureReferencePosition
l.obs2_JsensorPosition.value = JsensorPosition
l.obs2_sensorPosition.value = (0, 0, 0)
l.obs2_weight.value = (1., 0.)

l.obs2_featureObservedPosition.value = (5, 0)
l.obs2_featureReferencePosition.value = (0, 9999)

l.configurationOffset.recompute(0)
print l.configurationOffset.value
