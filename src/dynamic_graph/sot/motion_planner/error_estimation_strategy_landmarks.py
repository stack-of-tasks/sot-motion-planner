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

from dynamic_graph.sot.motion_planner import Localizer

from dynamic_graph.sot.motion_planner.error_estimator_strategy \
    import ErrorEstimationStrategy

class MotionCaptureErrorEstimationStrategy(ErrorEstimationStrategy):
    """
    Use a landmark based localization system to estimate the error.
    """

    """
    CORBA server used to received the perceived positions.
    """
    corba = None

    def __init__(self, feetFollowerWithCorrection, robot, corba = None):
        ErrorEstimationStrategy.__init__(self,
                                         robot, feetFollowerWithCorrection)

        # Create CORBA server if required.
        if not corba:
            corba = CorbaServer('corba_server')
        self.corba = corba

        # Select, X Y, yaw only!
        correctedDofs = (1., 1., 0., 0., 0., 1.) + 30 * (0.,)

        self.errorEstimator = Localizer('localizer')

        #self.errorEstimator.add_landmark_observation(obsName+str(i))
        #self.errorEstimator.signal(obsName + '_JfeatureReferencePosition').value = 
        #self.errorEstimator.signal(obsName + '_JsensorPosition').value = 
        #self.errorEstimator.signal(obsName + '_weight').value =

        #self.errorEstimator.signal(obsName + '_featureObservedPosition').value =
        #self.errorEstimator.signal(obsName + '_featureReferencePosition').value =

        # Select (x, y, yaw) only!
        #self.errorEstimator.signal(obsName + '_correctedDofs').value = correctedDofs

    def __str__(self):
        return "error estimation using landmarks"

    def start(self):
        pass

    def interactiveStart(self):
        pass


__all__ = ["MotionCaptureErrorEstimationStrategy"]
