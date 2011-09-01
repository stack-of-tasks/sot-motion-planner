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

from math import cos, sin, atan2

from dynamic_graph.sot.core import RobotSimu

from dynamic_graph import plug
from dynamic_graph.sot.core import FeatureGeneric, Task, RobotSimu
from dynamic_graph.sot.motion_planner.feet_follower import \
    FeetFollowerWithCorrection, Randomizer, ErrorEstimator
from dynamic_graph.sot.motion_planner.feet_follower_graph import \
    FeetFollowerGraph

from dynamic_graph.corba_server import CorbaServer

class FeetFollowerGraphWithCorrection(FeetFollowerGraph):
    """
    Enable online rectification of an existing FeetFollowerGraph using
    robot localization.

    This class is a decorator over a FeetFollowerGraph.


    It also takes as input an error estimation strategy. By default,
    it uses the motion capture system to estimate the position error.
    """

    """
    Uncorrected graph
    """
    feetFollowerGraph = None

    """
    Initial, uncorrected trajectory.
    """
    referenceTrajectory = None

    """
    Define the maximum correction.

    This has been empirically validated on HRP-2 14.

    This stores the maximum distance between the original and
    corrected footsteps (x, y in meters and rotation theta in rad).
    """
    (maxX, maxY, maxTheta) = (0.04, 0.04, 0.1)

    """
    Are we in simulation or in OpenHRP?
    """
    onRobot = None

    """
    Strategy used to compute the position error.

    This must be an instance of a subclass of ErrorEstimationStrategy.
    """
    errorEstimationStrategy = None

    def __str__(self):
        msg = \
            "* Is this the robot? "  + str(self.onRobot) + "\n" \
            "* Error estimation strategy: " + \
            str(self.errorEstimationStrategy) + "\n" + \
            "Safe limits: %f %f %f\n" % (self.maxX, self.maxY, self.maxTheta) \
            + "\n"
        if (self.onRobot):
            msg += "Please type 'f.start()' to start walking.\n"
        return msg

    def __init__(self, robot, solver,
                 feetFollowerGraph,
                 errorEstimationStrategyType,
                 maxX = 0.04,
                 maxY = 0.04,
                 maxTheta = 0.1):
        """
        To setup a correction, an initial feet follower graph
        is required.
        """

        self.feetFollowerGraph = feetFollowerGraph

        # Fill inherited attributes.
        self.feetFollower = feetFollowerGraph.feetFollower

        self.postureTask = feetFollowerGraph.postureTask
        self.postureFeature = feetFollowerGraph.postureFeature
        self.postureError = feetFollowerGraph.postureError
        self.trace = feetFollowerGraph.trace

        self.solver = solver
        self.robot = robot


        # Fill local attributes.
        self.maxX = maxX
        self.maxY = maxY
        self.maxTheta = maxTheta
        self.onRobot = type(self.robot.device) != RobotSimu

        # Store the reference trajectory (uncorrected).
        self.referenceTrajectory = self.feetFollower

        # Create the correction entity.
        self.feetFollower = FeetFollowerWithCorrection('correction')

        # Set the reference trajectory.
        self.feetFollower.setReferenceTrajectory(self.referenceTrajectory.name)

        # Set the safety limits.
        self.feetFollower.setSafetyLimits(self.maxX, self.maxY, self.maxTheta)

        # Replug.
        plug(self.feetFollower.zmp, self.robot.device.zmp)
        plug(self.feetFollower.com, self.robot.featureComDes.errorIN)
        plug(self.feetFollower.signal('left-ankle'),
             self.robot.features['left-ankle'].reference)
        plug(self.feetFollower.signal('right-ankle'),
             self.robot.features['right-ankle'].reference)
        plug(self.feetFollower.signal('waistYaw'),
             self.robot.features['waist'].reference)

        # Replug velocities.
        plug(self.feetFollower.comVelocity, self.robot.comTask.errorDot)
        for op in ['left-ankle', 'right-ankle']:
            plug(self.feetFollower.signal(op + 'Velocity'),
                 self.robot.tasks[op].errorDot)
        plug(self.feetFollower.waistYawVelocity,
             self.robot.tasks['waist'].errorDot)

        # Setup error estimation strategy.
        self.errorEstimationStrategy = errorEstimationStrategyType(self, robot)

        plug(self.robot.dynamic.signal(
                self.errorEstimationStrategy.localizationPlannedBody),
             self.feetFollower.position)

        # Plug the estimated error into the correction entity.
        plug(self.errorEstimationStrategy.errorEstimator.error,
             self.feetFollower.offset)

    def start(self, beforeStart = None):
        if self.onRobot:
            start = self.errorEstimationStrategy.start
        else:
            start = self.errorEstimationStrategy.interactiveStart

        if start():
            self.robot.comTask.controlGain.value = 180.
            self.robot.tasks['left-ankle'].controlGain.value = 180.
            self.robot.tasks['right-ankle'].controlGain.value = 180.
            self.feetFollowerGraph.postureTask.controlGain.value = 180.
            self.robot.tasks['waist'].controlGain.value = 180.
            self.feetFollowerGraph.setupTrace()
            if beforeStart:
                beforeStart()
            self.feetFollowerGraph.trace.start()
            self.feetFollower.start()
        else:
            print("failed to start")

    def setupTrace(self):
        self.feetFollowerGraph.setupTrace()
        signals = ['com', 'zmp', 'left-ankle', 'right-ankle', 'waistYaw']
        for s in signals:
            self.trace.add(self.referenceTrajectory.name + '.' + s,
                           self.referenceTrajectory.name + '-' + s)

            self.robot.device.after.addSignal(self.referenceTrajectory.name
                                              + '.' + s)
            self.robot.device.after.addSignal(self.feetFollower.name + '.' + s)

        self.trace.add(self.feetFollower.name + '.' + 'offset',
                       self.feetFollower.name + '-' + 'offset')
        self.robot.device.after.addSignal(self.feetFollower.name + '.' +
                                     'offset')

        self.trace.add(self.errorEstimationStrategy.errorEstimator.name +
                       '.' + 'error',
                       self.errorEstimationStrategy.errorEstimator.name +
                       '-' + 'error')
        self.robot.device.after.addSignal(
            self.errorEstimationStrategy.errorEstimator.name + '.' + 'error')

        self.trace.add(self.errorEstimationStrategy.errorEstimator.name
                       + '.' + 'dbgPositionWorldFrame',
                       self.errorEstimationStrategy.errorEstimator.name
                       + '-' + 'dbgPositionWorldFrame')
        self.robot.device.after.addSignal(
            self.errorEstimationStrategy.errorEstimator.name
            + '.' + 'dbgPositionWorldFrame')

        self.trace.add(self.errorEstimationStrategy.errorEstimator.name
                       + '.' + 'dbgPlanned',
                       self.errorEstimationStrategy.errorEstimator.name
                       + '-' + 'dbgPlanned')
        self.robot.device.after.addSignal(
            self.errorEstimationStrategy.errorEstimator.name
            + '.' + 'dbgPlanned')

        self.trace.add(self.errorEstimationStrategy.errorEstimator.name
                       + '.' + 'dbgIndex',
                       self.errorEstimationStrategy.errorEstimator.name
                       + '-' + 'dbgIndex')
        self.robot.device.after.addSignal(
            self.errorEstimationStrategy.errorEstimator.name + '.' + 'dbgIndex')

        self.trace.add(self.corba.name
         + '.' + self.errorEstimationStrategy.localizationPerceivedBody,
                       self.corba.name
         + '-' + self.errorEstimationStrategy.localizationPerceivedBody)
        self.robot.device.after.addSignal(
            self.corba.name + '.' +
            self.errorEstimationStrategy.localizationPerceivedBody)

        self.trace.add(self.robot.device.name + '.' + 'state',
                       self.robot.device.name + '-' + 'state')
        self.robot.device.after.addSignal(
            self.robot.device.name + '.' + 'state')

        self.trace.add(solver.sot.name + '.' + 'control',
                       solver.sot.name + '-' + 'control')
        self.robot.device.after.addSignal(solver.sot.name + '.' + 'control')


# Only export FeetFollowerGraphWithCorrection.
__all__ = ["FeetFollowerGraphWithCorrection"]
