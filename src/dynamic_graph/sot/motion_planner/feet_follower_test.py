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

from dynamic_graph.sot.dynamics.tools import *
from __main__ import robot, solver

from dynamic_graph.sot.core import \
    FeatureGeneric, Task, MatrixConstant, RobotSimu
from dynamic_graph.sot.motion_planner import FeetFollowerFromFile, FeetFollowerAnalyticalPg, PostureError, WaistPositionEstimator
from dynamic_graph.tracer_real_time import TracerRealTime
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas, Hrp2Jrl
from dynamic_graph.sot.dynamics.dynamic_hrp2 import DynamicHrp2


def translationToSE3(t):
    return ((1., 0., 0., t[0]),
            (0., 1., 0., t[1]),
            (0., 0., 1., t[2]),
            (0., 0., 0., 1.  ))

def oneVector(i):
    r = [0.,] * 36
    r[i] = 1.
    return tuple(r)

class Follower:
    feetFollower = None

    postureTask = None
    postureFeature = None
    postureFeatureDes = None
    postureError = None

    def __init__(self):
        self.feetFollower = FeetFollowerFromFile('feet-follower')
        #self.feetFollower = FeetFollowerAnalyticalPg('feet-follower')

        # Setup feet to ankle transformation.
        anklePosL = robot.dynamic.getAnklePositionInFootFrame()
        anklePosR = (anklePosL[0], -anklePosL[1], anklePosL[2])

        self.feetFollower.setLeftFootToAnkle(translationToSE3(anklePosL))
        self.feetFollower.setRightFootToAnkle(translationToSE3(anklePosR))

        # Setup initial robot position and com.
        #self.feetFollower.setInitialLeftAnklePosition(
        #robot.dynamic.signal('left-ankle').value)
        #self.feetFollower.setInitialRightAnklePosition(
        #robot.dynamic.signal('right-ankle').value)

        self.feetFollower.setInitialLeftAnklePosition(
            robot.features['left-ankle'].reference.value)
        self.feetFollower.setInitialRightAnklePosition(
            robot.features['right-ankle'].reference.value)

        self.feetFollower.setComZ(robot.dynamic.com.value[2])
        #self.feetFollower.setComZ(0.814)

        self.feetFollower.readTrajectory(
            # '/tmp'
            "/home/nddang/src/sotpy/sot-motion-planner/tests/local_stepper_test"
            )
        #'/home/thomas/profiles/laas/src/unstable/sot/sot-motion-planner/tests/simple_trajectory')

        #self.feetFollower.pushStep((0.1, 0.1, -0.19, 0.))
        #self.feetFollower.pushStep((0.1, 0.1, 0.19, 0.))
        #self.feetFollower.pushStep((0.1, 0.1, -0.19, 0.))
        #self.feetFollower.pushStep((0.1, 0.1, 0.19, 0.))
        #self.feetFollower.pushStep((0.05, 0.24, -0.19, 0.))
        #self.feetFollower.pushStep((0.05, 0.24,  0.19, 0.))
        #self.feetFollower.pushStep((0.05, 0.24, -0.19, 0.))
        #self.feetFollower.pushStep((0.05, 0.24,  0.19, 0.))

        #self.feetFollower.generateTrajectory()

        # Lower the gains to reduce the initial velocity.
        robot.comTask.controlGain.value = 5.
        robot.tasks['left-ankle'].controlGain.value = 5.
        robot.tasks['right-ankle'].controlGain.value = 5.

        # Make sure the CoM is converging toward the starting
        # CoM of the trajectory.
        robot.featureComDes.errorIN.value = \
            (0., 0., robot.dynamic.com.value[2])
        robot.featureCom.selec.value = '111'

        # Plug the feet follower output signals.
        plug(self.feetFollower.zmp, robot.device.zmp)

        plug(self.feetFollower.com, robot.featureComDes.errorIN)
        plug(self.feetFollower.signal('left-ankle'),
             robot.features['left-ankle'].reference)
        plug(self.feetFollower.signal('right-ankle'),
             robot.features['right-ankle'].reference)

        # Initialize the posture task.
        self.postureTask = Task(robot.name + '_posture')
        self.postureFeature = FeatureGeneric(robot.name + '_postureFeature')
        self.postureFeatureDes = \
            FeatureGeneric(robot.name + '_postureFeatureDes')

        self.postureError = PostureError('PostureError')
        plug(robot.device.state, self.postureError.state)
        plug(self.postureError.error, self.postureFeature.errorIN)

        self.postureFeature.jacobianIN.value = self.computeJacobian()
        self.postureFeatureDes.errorIN.value = self.computeDesiredValue()

        self.postureFeature.sdes.value = self.postureFeatureDes

        self.postureTask.add(self.postureFeature.name)
        self.postureTask.controlGain.value = 1.

        if type(robot.device) == RobotSimu:
            solver.sot.push(robot.comTask.name)
            solver.sot.push(robot.tasks['left-ankle'].name)
            solver.sot.push(robot.tasks['right-ankle'].name)
        solver.sot.push(self.postureTask.name)

    def computeError(self):
        e = robot.device.state.value
        e_ = [e[3], e[4], e[5]]
        offset = 6 + 2 * 6
        for i in xrange(len(e) - offset):
            e_.append(e[offset + i])
        return tuple(e_)

    def computeDesiredValue(self):
        e = robot.halfSitting
        e_ = [e[3], e[4], e[5]]
        offset = 6 + 2 * 6
        for i in xrange(len(e) - offset):
            e_.append(e[offset + i])
        return tuple(e_)


    def computeJacobian(self):
        j = []

        for i in xrange(36):
            if i == 3 or i == 4 or i == 5 or i >= 6 + 2 * 6:
                j.append(oneVector(i))
        return tuple(j)

    def canStart(self):
        securityThreshold = 1e-3
        return (robot.comTask.error.value <=
                (securityThreshold,) * len(robot.comTask.error.value)

                and robot.tasks['left-ankle'].error.value <=
                (securityThreshold,)
                * len(robot.tasks['left-ankle'].error.value)

                and robot.tasks['right-ankle'].error.value <=
                (securityThreshold,)
                * len(robot.tasks['right-ankle'].error.value)

                and self.postureTask.error.value <=
                (securityThreshold,) * len(self.postureTask.error.value))

    def start(self):
        if not self.canStart():
            print("Robot has not yet converged to the initial position,"
                  " please wait and try again.")
            return
        robot.comTask.controlGain.value = 180.
        robot.tasks['left-ankle'].controlGain.value = 180.
        robot.tasks['right-ankle'].controlGain.value = 180.
        self.postureTask.controlGain.value = 180.
        self.feetFollower.start()


f = Follower()

dyn2 =  DynamicHrp2('dyn2')
modelDir ='/local/nddang/profiles/sotpy/install/unstable/share/hrp2_14'
xmlDir = '/local/nddang/profiles/sotpy/install/unstable/share/hrp2_14'
modelName = 'HRP2JRLmainsmall.wrl'
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'
dyn2.setFiles(modelDir, modelName,
              specificitiesPath, jointRankPath)

dyn2.parse()
plug(robot.device.sensorState, dyn2.position)
dyn2.createOpPoint('left-ankle','left-ankle')
dyn2.createOpPoint('right-ankle','right-ankle')
dimension = dyn2.getDimension()
dyn2.velocity.value = dimension*(0.,)
dyn2.acceleration.value = dimension*(0.,)


trace = TracerRealTime('trace')
trace.setBufferSize(2**20)
trace.open('/tmp/','feet_follower_','.dat')

estimator = WaistPositionEstimator("estimator")
plug(dyn2.signal('left-ankle') ,estimator.leftAnkle)
plug(dyn2.signal('right-ankle'),estimator.rightAnkle)
plug(robot.device.forceRLEG     ,estimator.forceRleg)
plug(robot.device.forceLLEG     ,estimator.forceLleg)
plug(robot.device.state, estimator.stateIn)
robot.dynamic.position.unplug()
plug(estimator.stateOut, robot.dynamic.position)

# Feature sdes input.
trace.add('feet-follower.com', 'com')
trace.add('feet-follower.zmp', 'zmp')
trace.add('feet-follower.left-ankle', 'left-ankle')
trace.add('feet-follower.right-ankle', 'right-ankle')

trace.add('dyn2.left-ankle', 'left-ankle-dyn2')
trace.add('dyn2.right-ankle', 'right-ankle-dyn2')
trace.add('robot_dynamic.position', 'position')
trace.add('estimator.stateOut', 'stateOut')

# Tasks error sdes input.
trace.add(robot.comTask.name + '.error', 'errorCom')
trace.add(robot.tasks['left-ankle'].name + '.error', 'errorLa')
trace.add(robot.tasks['right-ankle'].name + '.error', 'errorRa')
trace.add(f.postureTask.name + '.error', 'errorPosture')
trace.add(robot.device.name + ".state", 'state')
trace.add(robot.device.name + ".sensorState", 'sensorState')
trace.add(robot.device.name + ".forceRLEG", 'forceRLEG')
trace.add(robot.device.name + ".forceLLEG", 'forceLLEG')
trace.add(robot.device.name + ".forceRARM", 'forceRARM')
trace.add(robot.device.name + ".forceLARM", 'forceLARM')
trace.add("estimator.z","z")


# Recompute trace.triger at each iteration to enable tracing.
robot.device.after.addSignal('trace.triger')
robot.device.after.addSignal('dyn2.left-ankle')
robot.device.after.addSignal('dyn2.right-ankle')
robot.device.after.addSignal('estimator.z')

trace.start()
