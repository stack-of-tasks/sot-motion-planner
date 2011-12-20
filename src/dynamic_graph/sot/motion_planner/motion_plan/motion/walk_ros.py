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

import numpy as np

from dynamic_graph import plug
from dynamic_graph.sot.core import FeatureGeneric, TaskPD, Task, RobotSimu
from dynamic_graph.sot.core.feature_position import FeaturePosition

from dynamic_graph.sot.motion_planner.feet_follower \
    import FeetFollowerRos, FeetFollowerWithCorrection

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.motion.abstract import *

class MotionWalkRos(Motion):
    yaml_tag = u'walk-ros'

    initialGain = 175.

    def __init__(self, motion, yamlData, defaultDirectories):
        checkDict('ros-parameter', yamlData)
        checkDict('interval', yamlData)

        Motion.__init__(self, motion, yamlData)

        self.name = id(yamlData)

        self.rosParameter = yamlData['ros-parameter']
        self.rosLeftAnkle = yamlData['ros-left-ankle']

        motion.ros.rosImport.add('matrixHomoStamped',
                                 'left-ankle', self.rosLeftAnkle)

        self.initialLeftAnklePosition = \
            self.robot.features['left-ankle'].reference.value
        self.initialRightAnklePosition = \
            self.robot.features['right-ankle'].reference.value

        self.feetFollower = FeetFollowerRos(
            "{0}_feet_follower".format(self.name))
        self.setAnklePosition()
        self.setInitialFeetPosition()
        print("Parsing trajectory...")
        self.feetFollower.parseTrajectory(self.rosParameter)
        print("done.")

        plug(self.feetFollower.signal('left-ankle'),
             motion.ros.rosImport.signal('left-ankle'))

        # Center of mass features and task.
        (self.featureCom, self.featureComDes, self.comTask) = \
            self.robot.createCenterOfMassFeatureAndTask(
            '{0}_feature_com'.format(self.name),
            '{0}_feature_ref_com'.format(self.name),
            '{0}_task_com'.format(self.name),
            selec = '111',
            gain = self.initialGain)

        # Make sure the CoM is converging toward the starting
        # CoM of the trajectory.
        self.featureComDes.errorIN.value = \
            (0., 0., self.robot.dynamic.com.value[2])


        # Operational points features/tasks.
        self.features = dict()
        self.tasks = dict()
        for op in ['left-ankle', 'right-ankle', 'waist']:
            (self.features[op], self.tasks[op]) = \
                self.robot.createOperationalPointFeatureAndTask(
                op, '{0}_feature_{1}'.format(self.name, op),
                '{0}_task_{1}'.format(self.name, op),
                gain = self.initialGain)

        # Initialize the waist yaw task.
        self.features['waist'].selec.value = '111000'
        self.tasks['waist'].controlGain.value = self.initialGain

        # Upper body posture
        self.posture = FeatureGeneric("{0}_posture".format(self.name))
        self.postureDes = FeatureGeneric("{0}_posture_des".format(self.name))
        self.posture.sdes.value = self.postureDes

        plug(self.robot.device.state, self.posture.errorIN)
        self.posture.jacobianIN.value = self.jacobianPosture()
        self.posture.selec.value = '1' * len(self.robot.halfSitting)

        self.postureTask = Task("{0}_posture_task".format(self.name))
        self.postureTask.add(self.posture.name)
        self.postureTask.controlGain.value = self.initialGain

        # Create the correction entity.
        self.correction = FeetFollowerWithCorrection(
            '{0}_correction'.format(self.name))
        # Set the reference trajectory.
        self.correction.setReferenceTrajectory(self.feetFollower.name)

        # Set the safety limits.
        self.correction.setSafetyLimits(motion.maxX, motion.maxY, motion.maxTheta)

        self.correction.setFootsteps(
            2., # 2 seconds per step
            makeFootsteps([]))


        # Plug the feet follower output signals.
        plug(self.correction.zmp, self.robot.device.zmp)

        plug(self.correction.com, self.featureComDes.errorIN)
        plug(self.correction.signal('left-ankle'),
             self.features['left-ankle'].reference)
        plug(self.correction.signal('right-ankle'),
             self.features['right-ankle'].reference)
        plug(self.correction.waistYaw, self.features['waist'].reference)
        plug(self.correction.posture, self.postureDes.errorIN)


        # Plug velocities into TaskPD.
        plug(self.correction.comVelocity, self.comTask.errorDot)
        for op in ['left-ankle', 'right-ankle']:
            plug(self.correction.signal(op + 'Velocity'),
                 self.tasks[op].errorDot)
        plug(self.correction.waistYawVelocity,
             self.tasks['waist'].errorDot)

        # By default set error to zero.
        self.correction.offset.value = (0., 0., 0.)

        #FIXME: this should match the ROS topic error frame_id.
        # Unfortunately we do not have any way to retrieve it for now.
        plug(self.robot.dynamic.signal('left-ankle'),
             self.correction.position)


        #FIXME: HRP-2 specific
        unlockedDofsRleg = []
        unlockedDofsLleg = []
        unlockedDofsUpperBody = []
        for i in xrange(6):
            unlockedDofsRleg.append(6 + i)
            unlockedDofsLleg.append(6 + 6 + i)

        for i in xrange(len(self.robot.halfSitting) - 6 - 12):
            unlockedDofsUpperBody.append(6 + 12 + i)


        # Push the tasks into supervisor.
        motion.supervisor.addFeetFollowerStartCall(
            self.correction.name,
            self.interval[0])

        motion.supervisor.addTask(self.postureTask.name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 4,
                                  tuple(unlockedDofsUpperBody))
        motion.supervisor.addTask(self.comTask.name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 3,
                                  tuple(self.extraUnlockedDofs))
        motion.supervisor.addTask(self.tasks['left-ankle'].name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 2,
                                  tuple(unlockedDofsLleg))
        motion.supervisor.addTask(self.tasks['right-ankle'].name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 1,
                                  tuple(unlockedDofsRleg))
        motion.supervisor.addTask(self.tasks['waist'].name,
                                  self.interval[0], self.interval[1],
                                  self.priority,
                                  ())

    def __str__(self):
        fmt = "walking motion ROS"
        return fmt

    def setupTrace(self, trace):
        print("setup trace for walk_ros")
        # Feet follower
        for s in ['zmp', 'waist',
                  'com', 'left-ankle', 'right-ankle', 'waistYaw',
                  'comVelocity', 'left-ankleVelocity',
                  'right-ankleVelocity', 'waistYawVelocity']:
            self.robot.addTrace(self.feetFollower.name, s)
            self.robot.addTrace(self.correction.name, s)

    def canStart(self):
        return True #FIXME:

    def setAnklePosition(self):
        # Setup feet to ankle transformation.
        anklePosL = self.robot.dynamic.getAnklePositionInFootFrame()
        anklePosR = (anklePosL[0], -anklePosL[1], anklePosL[2])

        self.feetFollower.setLeftFootToAnkle(translationToSE3(anklePosL))
        self.feetFollower.setRightFootToAnkle(translationToSE3(anklePosR))

    def setInitialFeetPosition(self):
        self.feetFollower.setInitialLeftAnklePosition(
            self.initialLeftAnklePosition)
        self.feetFollower.setInitialRightAnklePosition(
            self.initialRightAnklePosition)

    def jacobianPosture(self):
        size = len(self.robot.halfSitting)
        J = np.matrix(np.identity(size))

        #FIXME: not generic enough
        for i in xrange(6 + 12):
            J[i, i] = 0.
        return matrixToTuple(J)
