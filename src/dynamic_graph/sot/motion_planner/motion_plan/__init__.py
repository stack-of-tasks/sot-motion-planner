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
import logging
import yaml
import numpy as np
import textwrap

from dynamic_graph import plug
from dynamic_graph.ros import Ros
from dynamic_graph.sot.motion_planner.feet_follower import \
    Supervisor
from dynamic_graph.sot.motion_planner.feet_follower_graph_with_correction \
    import FeetFollowerGraphWithCorrection

from dynamic_graph.sot.motion_planner.math import *

from dynamic_graph.corba_server import CorbaServer

from dynamic_graph.sot.motion_planner.motion_plan.control import *
from dynamic_graph.sot.motion_planner.motion_plan.environment import *
from dynamic_graph.sot.motion_planner.motion_plan.error_strategy import *
from dynamic_graph.sot.motion_planner.motion_plan.motion import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

def initializeLogging():
    logger = logging.getLogger('motion-plan')
    logger.setLevel(logging.INFO)

    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    # create formatter
    fmt = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    formatter = logging.Formatter(fmt)

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)
    return logger


class MotionPlan(object):
    name = 'motionPlan'

    robot = None
    solver = None

    corba = None
    ros = None

    plan = None
    duration = 0
    motion = []
    control = []
    footsteps = []
    environment = {}

    postureFeature = None
    postureTask = None

    started = False

    maxX = FeetFollowerGraphWithCorrection.maxX
    maxY = FeetFollowerGraphWithCorrection.maxY
    maxTheta = FeetFollowerGraphWithCorrection.maxTheta

    def __init__(self, filename, robot, solver, defaultDirectories,
                 logger = None):
        if not logger:
            logger = initializeLogging()

        self.defaultDirectories = defaultDirectories

        self.robot = robot
        self.solver = solver
        self.logger = logger

        self.logger.info('loading motion plan file \'{0}\''.format(filename))
        self.plan = yaml.load(
            open(searchFile(filename, defaultDirectories), "r"))

        self.duration = float(self.plan['duration'])

        # Middleware proxies.
        self.corba = CorbaServer('corba_server')
        self.ros = Ros(self.robot)

        # Posture feature.
        self.postureFeature = FeaturePosture(
            '{0}_postureFeature'.format(self.name))
        plug(self.robot.device.state, self.postureFeature.state)

        posture = list(self.robot.halfSitting)
        self.postureFeature.setPosture(tuple(posture))
        for i in xrange(len(posture) - 6):
            self.postureFeature.selectDof(i + 6, True)

        self.postureTask = Task('{0}_posture'.format(self.name))
        self.postureTask.add(self.postureFeature.name)
        self.postureTask.controlGain.value = 1.

        # Supervisor.
        self.supervisor = Supervisor('supervisor')
        self.robot.device.after.addSignal(self.supervisor.name + '.trigger')
        self.supervisor.setSolver(self.solver.sot.name)
        self.supervisor.setPostureFeature(self.postureFeature.name)

        # Add the posture task to the supervisor.
        self.supervisor.addTask(
            self.postureTask.name, 0., self.duration, 100, ())


        # Load plan.
        self.logger.debug('loading environment')
        self.loadEnvironment()
        self.logger.debug('loading motion elements')
        self.loadMotion()
        self.logger.debug('loading control elements')
        self.loadControl()

        # For now, only 1 feet follower is allowed (must start at t=0).
        feetFollowerElement = find(lambda e: type(e) == MotionWalk, self.motion)
        self.hasControl = len(self.control) > 0

        controlRosElement = find(lambda e: type(e) == ControlRos, self.control)

        # Plug motion signals which depend on control.
        for m in self.motion:
            if type(m) == MotionVisualPoint:
                # FIXME: this is so wrong.
                plug(self.ros.rosExport.signal(m.objectName),
                     m.vispPointProjection.cMo)
                plug(self.ros.rosExport.signal(m.objectName + 'Timestamp'),
                     m.vispPointProjection.cMoTimestamp)

        # If control elements are planned, correct the execution using
        # localization information.
        if self.hasControl:
            for i in xrange(len(self.motion)):
                if type(self.motion[i]) == MotionWalkRos and controlRosElement:
                    print("plugging error estimation from ROS")
                    plug(self.ros.rosExport.signal(controlRosElement.signal),
                         self.motion[i].correction.offset)

                if type(self.motion[i]) != MotionWalk:
                    continue
                # Add correction graph.
                self.motion[i].correction = FeetFollowerGraphWithCorrection(
                    robot, solver, self.motion[i].feetFollower,
                    MotionPlanErrorEstimationStrategy,
                    maxX = self.maxX, maxY = self.maxY,
                    maxTheta = self.maxTheta)
                self.motion[i].correction.errorEstimationStrategy.motionPlan = \
                    self
                #FIXME: not generic enough
                self.motion[i].correction.feetFollower.setFootsteps(
                    2., # 2 seconds per step
                    makeFootsteps(feetFollowerElement.footsteps))

                self.motion[i].correction.setupTrace()

                # Start the correction when needed.
                self.supervisor.addFeetFollowerStartCall(
                    self.motion[i].correction.feetFollower.name,
                    self.motion[i].interval[0])

        self.logger.debug('motion plan created with success')

    def loadEnvironment(self):
        if not 'environment' in self.plan:
            return

        for obj in self.plan['environment']:
            checkDict('object', obj)
            checkDict('name', obj['object'])
            self.environment[obj['object']['name']] = \
                EnvironmentObject(self, obj['object'])
            self.logger.debug('adding object \'{0}\''.format(obj['object']['name']))

    def loadMotion(self):
        if not 'motion' in self.plan or not self.plan['motion']:
            return

        if 'maximum-correction-per-step' in self.plan:
            self.maxX = self.plan['maximum-correction-per-step']['x']
            self.maxY = self.plan['maximum-correction-per-step']['y']
            self.maxTheta = self.plan['maximum-correction-per-step']['theta']

        motionClasses = [MotionWalk, MotionJoint, MotionTask,
                         MotionVisualPoint, MotionWalkRos]

        for motion in self.plan['motion']:
            if len(motion.items()) != 1:
                raise RuntimeError('each motion should have only one type')
            (tag, data) = motion.items()[0]
            cls = find(lambda c: c.yaml_tag == tag, motionClasses)

            if not cls:
                raise RuntimeError('invalid motion element')
            self.motion.append(cls(self, data, self.defaultDirectories))
            self.logger.debug('adding motion element \'{0}\''.format(tag))

        for motion in self.motion:
            motion.setupTrace(self.robot.tracer)


    def loadControl(self):
        if not 'control' in self.plan or not self.plan['control']:
            return

        controlClasses = [ControlConstant, ControlMocap, ControlViSP,
                          ControlHueblob, ControlVirtualSensor, ControlRos]

        for control in self.plan['control']:
            if len(control.items()) != 1:
                raise RuntimeError('each control should have only one type')
            (tag, data) = control.items()[0]
            cls = find(lambda c: c.yaml_tag == tag, controlClasses)
            if not cls:
                raise RuntimeError('invalid control element')
            self.control.append(cls(self, data))
            self.logger.debug('adding control element \'{0}\''.format(tag))

    def __str__(self):
        res  = 'Motion:\n'
        res += '-------\n'
        for motion in self.motion:
            res += '\n\t'.join(textwrap.wrap('\t* {0}\n'.format(str(motion)))) + '\n'
        res += '\n'
        res += 'Control:\n'
        res += '--------\n'
        for control in self.control:
            res += '\n\t'.join(textwrap.wrap('\t* {0}\n'.format(str(control)))) + '\n'
        res += '\n\n'
        res += self.supervisor.display()
        return res

    def start(self):
        if self.started:
            self.logger.info('already started')
            return
        if not self.canStart():
            self.logger.info('failed to start')
            return
        self.started = True
        self.logger.info('execution starts')

        if self.hasControl:
            for i in xrange(len(self.motion)):
                if type(self.motion[i]) != MotionWalk:
                    continue
                self.motion[i].correction.errorEstimationStrategy.start(
                    interactive = False)

        self.robot.startTracer()

        # Remove default tasks and let the supervisor take over the
        # tasks management.
        self.solver.sot.clear()
        self.supervisor.start()
        self.supervisor.display()


    def stop(self):
        self.supervisor.stop()
        self.solver.sot.clear()
        self.robot.stopTracer()
        self.logger.info('motion_plan stopped')

    def canStart(self):
        return (reduce(lambda acc, c: c.canStart() and acc,
                      self.control, True) and
                # Search for motion elements starting at t = 0.
                reduce(lambda acc, c: (c.interval[0] == 0.
                                       or c.canStart()) and acc,
                       self.motion, True))
