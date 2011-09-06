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
from math import cos, sin, atan2
import numpy as np

from dynamic_graph.sot.core import RobotSimu

from dynamic_graph import plug
from dynamic_graph.sot.core import FeatureGeneric, Task, RobotSimu
from dynamic_graph.sot.motion_planner.feet_follower import \
    FeetFollowerWithCorrection, Randomizer, ErrorEstimator
from dynamic_graph.sot.motion_planner.feet_follower_graph import \
    FeetFollowerGraph

from dynamic_graph.sot.motion_planner.math import *

from dynamic_graph.corba_server import CorbaServer


class ErrorEstimationStrategy(object):
    """
    Error estimator entity.

    It can be either from an external localization system or a simulation
    entity returning random values.
    """

    """
    Error estimator entity computing the error.

    It must provide an error signal containing the current error.

    This error corresponds to the planned robot position in the real
    robot position frame.
    """
    errorEstimator = None

    """
    Robot executing the trajectory.
    """
    robot = None

    def __init__(self, robot, feetFollowerWithCorrection):
        self.robot = robot

    def start(self):
        """Start the error measurement."""
        pass

    def interactiveStart(self):
        """Start interactively the error measurement"""
        pass

class MotionCaptureErrorEstimationStrategy(ErrorEstimationStrategy):
    """
    Use the motion capture system as an external localization device.

    The last perceived position of a particular robot body is compared
    with its planned position to deduce a measurement of the position
    error.

    By default, this body is the left foot. It is preferable over other
    bodies such as the waist which position is modified by the stabilizer
    on robots such as HRP-2.

    The timestamp associated with the perception of the body position
    by the localization system is required to handle delays on the
    perception part.
    """


    """
    Body used for perception.

    The error is computed by comparing the body localizationPlannedBody
    in robot.dynamic and the position given by the localization system
    in corba.signal(localizationPerceivedBody).
    Additionnally, corba.signal(localizationPerceivedBodyTimestamp)
    provides the timestamp associated with the current perception.

    Note: only used when enableLocalization is true.
    """
    localizationPlannedBody = 'left-ankle'
    localizationPerceivedBody = 'left-foot'
    localizationPerceivedBodyTimestamp = 'left-footTimestamp'

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

        self.errorEstimator = ErrorEstimator('error_estimator')

        self.errorEstimator.setReferenceTrajectory(
            feetFollowerWithCorrection.referenceTrajectory.name)

        # FIXME: we use ankle position as foot position here
        # as Z does not matter.
        plug(feetFollowerWithCorrection.referenceTrajectory.signal
             (self.localizationPlannedBody),
             self.errorEstimator.planned)

    def __str__(self):
        return "error estimation using the motion capture system " + \
            " (tracked body: %s)" % self.localizationPerceivedBody


    def computeWorldTransformationFromFoot(self):
        """
        This methods makes the assumption that the robot is placed
        exactly at its starting position.

        By comparing the current localization with the starting
        position of the tracked body, it deduces the transformation
        between the motion capture system and the control framework.
        """
        self.corba.signal(self.localizationPerceivedBody).recompute(
            self.corba.signal(self.localizationPerceivedBody).time + 1)
        self.robot.dynamic.waist.recompute(self.robot.dynamic.waist.time + 1)

        mocapMfoot = XYThetaToHomogeneousMatrix(
            self.corba.signal(self.localizationPerceivedBody).value)
        sotMfoot = np.matrix(self.robot.dynamic.signal(
                self.localizationPlannedBody).value)

        # mocap position w.r.t sot frame
        sotMmocap = sotMfoot * np.linalg.inv(mocapMfoot)
        return matrixToTuple(sotMmocap)

    def start(self):
        """
        Initialize the motion capture system.

        Before calling this function, make sure:
        a. evart-to-client is launched and successfully track
          the body used to compute the error,
        b. the robot is placed at its starting position.
        """
        if len(self.corba.signals()) == 3:
            print ("evart-to-client not launched, abandon.")
            return False
        if len(self.corba.signal(self.localizationPerceivedBody).value) != 3:
            print ("waist not tracked, abandon.")
            return False

        self.sMm = self.computeWorldTransformationFromFoot()

        print("World transformation:")
        print(HomogeneousMatrixToXYZTheta(self.sMm))
        self.errorEstimator.setSensorToWorldTransformation(self.sMm)
        plug(self.corba.signal(self.localizationPerceivedBody),
             self.errorEstimator.position)
        plug(self.corba.signal(
                self.localizationPerceivedBody + 'Timestamp'),
             self.errorEstimator.positionTimestamp)
        print ("Initial error:")
        print (self.errorEstimator.error.value)
        return True

    def interactiveStart(self):
        while len(self.corba.signals()) == 3:
            raw_input("Press enter after starting evart-to-corba.")
        while len(self.corba.signal('left-foot').value) != 3:
            raw_input("Body not tracked...")
        return self.start()



class RandomErrorEstimationStrategy(ErrorEstimationStrategy):
    """
    Compute an error randomly.
    """

    def __init__(self, feetFollowerWithCorrection):
        ErrorEstimatorStrategy.__init__(self, feetFollowerWithCorrection)

        self.errorEstimator = Randomizer('r')
        self.errorEstimator.addSignal('error', 3)

    def start(self):
        return True

    def interactiveStart(self):
        return True

    def __str__(self):
        return "random error estimation"


# Only export FeetFollowerGraphWithCorrection.
__all__ = ["ErrorEstimationStrategy",
           "MotionCaptureErrorEstimationStrategy",
           "RandomErrorEstimationStrategy"]
