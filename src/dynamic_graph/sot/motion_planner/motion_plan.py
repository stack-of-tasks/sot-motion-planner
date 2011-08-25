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

import yaml

from math import cos, sin, atan2
import numpy as np

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import VirtualSensor

from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph
from dynamic_graph.sot.motion_planner.feet_follower_graph_with_correction \
    import FeetFollowerGraphWithCorrection
from dynamic_graph.sot.motion_planner import \
    ErrorEstimator, ErrorMerger

from dynamic_graph.sot.motion_planner.error_estimation_strategy \
    import ErrorEstimationStrategy, MotionCaptureErrorEstimationStrategy

from dynamic_graph.sot.motion_planner.math import *

from dynamic_graph.corba_server import CorbaServer

def addTrace(robot, trace, entityName, signalName):
    trace.add(entityName + '.' + signalName,
              entityName + '-' + signalName)
    robot.device.after.addSignal(entityName + '.' + signalName)

def convertToNPFootstepsStack(footsteps):
    slide2 = -0.760000
    res = []
    for step in footsteps:
        if len(res) == 0:
            slide1 = 0
        else:
            slide1 = -1.520000
        res.append((slide1, 0.24, 0.25, slide2,
                    step['x'], step['y'], step['theta']))
    return tuple(res)

def makeFootsteps(footsteps):
    res = []
    for step in footsteps:
        res.append(step['x'])
        res.append(step['y'])
        res.append(step['theta'])
    return tuple(res)

class MotionPlanErrorEstimationStrategy(ErrorEstimationStrategy):
    localizationPlannedBody = 'waist'
    errorEstimators = []

    # reference toward the motion plan, added after this object
    # construction and before starting the error estimation.
    motionPlan = None

    def __init__(self, feetFollowerWithCorrection, robot, corba = None):
        ErrorEstimationStrategy.__init__(self,
                                         robot, feetFollowerWithCorrection)

        # Create CORBA server if required.
        if not corba:
            corba = CorbaServer('corba_server')
        self.corba = corba

        self.errorEstimator = ErrorMerger('error_merger')
        self.feetFollowerWithCorrection = feetFollowerWithCorrection
        self.robot = robot

    def startVirtualSensor(self, control, name):
        I = ((1.,0.,0.,0.), (0.,1.,0.,0.), (0.,0.,1.,0.), (0.,0.,0.,1.))
        estimator = ErrorEstimator(name)
        estimator.setReferenceTrajectory(
            self.feetFollowerWithCorrection.referenceTrajectory.name)

        # FIXME: not generic enough.
        plug(self.feetFollowerWithCorrection.referenceTrajectory.waist,
             estimator.planned)

        estimator.setSensorToWorldTransformation(I)

        plug(control[2].position, estimator.position)
        plug(control[2].positionTimestamp, estimator.positionTimestamp)
        return estimator

    def computeWorldTransformationFromFoot(self,
                                           localizationPlannedBody,
                                           localizationPerceivedBody):
        """
        This methods makes the assumption that the robot is placed
        exactly at its starting position.

        By comparing the current localization with the starting
        position of the tracked body, it deduces the transformation
        between the motion capture system and the control framework.
        """
        self.corba.signal(localizationPerceivedBody).recompute(
            self.corba.signal(localizationPerceivedBody).time + 1)
        self.robot.dynamic.waist.recompute(self.robot.dynamic.waist.time + 1)

        mocapMfoot = XYThetaToHomogeneousMatrix(
            self.corba.signal(localizationPerceivedBody).value)
        sotMfoot = np.matrix(self.robot.dynamic.signal(
                localizationPlannedBody).value)

        # mocap position w.r.t sot frame
        sotMmocap = sotMfoot * np.linalg.inv(mocapMfoot)
        return matrixToTuple(sotMmocap)


    def startMocap(self, control, name):
        estimator = ErrorEstimator(name)
        estimator.setReferenceTrajectory(
            self.feetFollowerWithCorrection.referenceTrajectory.name)

        localizationPlannedBody = control[1]['tracked-body']
        localizationPerceivedBody = control[1]['perceived-body']

        # FIXME: we use ankle position as foot position here
        # as Z does not matter.
        plug(self.feetFollowerWithCorrection.referenceTrajectory.signal
             (localizationPlannedBody),
             estimator.planned)

        if len(self.corba.signals()) == 3:
            print ("evart-to-client not launched, abandon.")
            return False
        if len(self.corba.signal(localizationPerceivedBody).value) != 3:
            print ("waist not tracked, abandon.")
            return False

        sMm = self.computeWorldTransformationFromFoot(
            localizationPlannedBody,
            localizationPerceivedBody)

        estimator.setSensorToWorldTransformation(sMm)
        plug(self.corba.signal(localizationPerceivedBody),
             estimator.position)
        plug(self.corba.signal(
                localizationPerceivedBody + 'Timestamp'),
             estimator.positionTimestamp)
        return estimator

    def start(self):
        for control in self.motionPlan.control:
            name = 'error_estimator' + str(id(control))
            self.errorEstimator.addErrorEstimation(name)

            if control[0] == 'virtual-sensor':
                estimator = self.startVirtualSensor(control, name)
            elif control[0] == 'mocap':
                estimator = self.startMocap(control, name)
            elif control[0] == 'visp':
                raise NotImplementedError
            else:
                raise RuntimeError("invalid control element")
            plug(estimator.error,
                 self.errorEstimator.signal("error_" + name))
            self.errorEstimator.signal("weight_" + name).value = \
                (control[1]['weight'],)
            self.errorEstimators.append(estimator)

            if self.motionPlan.trace:
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'error')
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'dbgPositionWorldFrame')
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'dbgPlanned')
                addTrace(self.motionPlan.robot,
                         self.motionPlan.trace,
                         name, 'dbgIndex')

        if self.motionPlan.trace:
            addTrace(self.motionPlan.robot,
                     self.motionPlan.trace,
                     self.errorEstimator.name, 'error')
        return True

    def interactiveStart(self):
        mustWait = False

        for control in self.motionPlan.control:
            if control[0] == 'mocap':
                mustWait = True

        if mustWait:
            while len(self.corba.signals()) == 3:
                raw_input("Press enter after starting evart-to-corba.")
            while len(self.corba.signal('left-foot').value) != 3:
                raw_input("Body not tracked...")


        return self.start()

    def __str__(self):
        return "motion plan error estimation strategy"

class MotionPlan(object):
    robot = None
    solver = None

    feetFollower = None

    plan = None
    duration = 0
    motion = []
    control = []
    footsteps = []

    trace = None

    maxX = FeetFollowerGraphWithCorrection.maxX
    maxY = FeetFollowerGraphWithCorrection.maxY
    maxTheta = FeetFollowerGraphWithCorrection.maxTheta

    def __init__(self, filename, robot, solver):
        self.robot = robot
        self.solver = solver
        self.plan = yaml.load(open(filename, "r"))
        self.duration = float(self.plan['duration'])

        self.loadMotion()
        self.loadControl()

        # For now, only 1 feet follower is allowed (must start at t=0).
        if len(self.motion):
            if len(self.control):
                self.feetFollower = FeetFollowerGraphWithCorrection(
                    robot, solver, self.motion[0][1],
                    MotionPlanErrorEstimationStrategy,
                    maxX = self.maxX, maxY = self.maxY,
                    maxTheta = self.maxTheta)
                self.feetFollower.errorEstimationStrategy.motionPlan = self
                #FIXME: not enough generic
                self.feetFollower.feetFollower.setFootsteps(
                    2., makeFootsteps(self.footsteps))
            else:
                self.feetFollower = self.motion[0][1]
        else:
            self.feetFollower = None

    def loadMotionWalk(self, motionWalk):
        steps = convertToNPFootstepsStack(motionWalk['footsteps'])
        #FIXME: handle multiple walk movement.
        self.footsteps = motionWalk['footsteps']
        waistFile = None
        if 'waist-trajectory' in motionWalk:
            waistFile = motionWalk['waist-trajectory']
        feetFollower = FeetFollowerAnalyticalPgGraph(
            self.robot, self.solver, steps,
            waistFile=waistFile)
        self.motion.append([motionWalk, feetFollower])
        print("successfully loaded walk motion")
        self.trace = feetFollower.trace

    def loadMotionTask(self, motionTask):
        if not 'interval' in motionTask or len(motionTask['interval']) != 2:
            raise RuntimeErrror('interval is missing')

        interval = motionTask['interval']
        type_ = motionTask['type']

        if type_ != 'feature-point-6d' and type_ != 'feature-com':
            raise NotImplementedError
        # Cannot change the stack dynamically for now.
        if interval[0] != 0 and interval[1] != self.duration:
            raise NotImplementedError

        gain = motionTask['gain']
        reference = motionTask['reference']

        if type_ == 'feature-point-6d':
            body = motionTask['operational-point']

            self.robot.tasks[body].controlGain.value = gain
            if reference == 'static':
                self.robot.features[body]._reference.position.value = \
                    self.robot.dynamic.signal(body).value
            else:
                self.robot.features[body]._reference.position.value = \
                    ((1., 0., 0., reference['x']),
                     (0., 1., 0., reference['y']),
                     (0., 0., 1., reference['z']),
                     (0., 0., 0., 1.),)
            self.solver.sot.push(self.robot.tasks[body].name)
        elif type_ == 'feature-com':
            self.robot.comTask.controlGain.value = gain
            if reference == 'static':
                self.robot.featureComDes.errorIN.value = \
                    self.robot.dynamic.com.value
            else:
                self.robot.featureComDes.position.value = \
                    (reference['x'], reference['y'])
            self.solver.sot.push(self.robot.comTask.name)

        #self.motion.append([motionTask])
        print("successfully loaded task")


    def loadMotion(self):
        if not 'motion' in self.plan or not self.plan['motion']:
            return

        if 'maximum-correction-per-step' in self.plan:
            self.maxX = self.plan['maximum-correction-per-step']['x']
            self.maxY = self.plan['maximum-correction-per-step']['y']
            self.maxTheta = self.plan['maximum-correction-per-step']['theta']

        for motion in self.plan['motion']:
            if 'walk' in motion:
                self.loadMotionWalk(motion['walk'])
            elif 'task' in motion:
                self.loadMotionTask(motion['task'])
            else:
                raise "invalid motion element"

    def loadControlMocap(self, mocap):
        self.control.append(['mocap', mocap])
        print("successfully loaded motion capture control")

    def loadControlVisp(self, visp):
        #self.control.append([FIXME])
        raise NotImplementedError

    def loadControlHueblob(self, hueblob):
        #self.control.append([FIXME])
        raise NotImplementedError

    def loadControlVirtualSensor(self, virtualSensorData):
        virtualSensor = VirtualSensor(
            'virtualSensor' + str(id(virtualSensorData)))

        #FIXME: should be more generic.
        plug(self.motion[0][1].feetFollower.waist,
             virtualSensor.expectedRobotPosition)
        plug(self.robot.dynamic.waist,
             virtualSensor.robotPosition)

        reference = virtualSensorData['obstacle-position']['planned']
        position = virtualSensorData['obstacle-position']['estimated']

        virtualSensor.expectedObstaclePosition.value = \
                    ((1., 0., 0., reference['x']),
                     (0., 1., 0., reference['y']),
                     (0., 0., 1., reference['z']),
                     (0., 0., 0., 1.),)
        virtualSensor.obstaclePosition.value = \
                    ((1., 0., 0., position['x']),
                     (0., 1., 0., position['y']),
                     (0., 0., 1., position['z']),
                     (0., 0., 0., 1.),)
        self.control.append(['virtual-sensor',
                             virtualSensorData, virtualSensor])

        if self.trace:
            addTrace(self.robot, self.trace, virtualSensor.name, 'position')


    def loadControl(self):
        if not 'control' in self.plan or not self.plan['control']:
            return
        for control in self.plan['control']:
            if 'mocap' in control:
                self.loadControlMocap(control['mocap'])
            elif 'visp' in control:
                self.loadControlVisp(control['visp'])
            elif 'hueblob' in control:
                self.loadControlHueblob(control['hueblob'])
            elif 'virtual-sensor' in control:
                self.loadControlVirtualSensor(control['virtual-sensor'])
            else:
                raise "invalid control element"

    def displayMotion(self):
        print('Motion:')
        print('-------')
        for motion in self.motion:
            print('\t* ' + str(motion[1]))
        print('')
        print('Control:')
        print('--------')
        for control in self.control:
            print('\t* ' + str(control[0]))
        print('')
        print(self.feetFollower)
        print('')
        print('')

    def start(self):
        if self.feetFollower:
            self.feetFollower.start()

    def canStart(self):
        if self.feetFollower:
            return self.feetFollower.canStart()
        else:
            return True
