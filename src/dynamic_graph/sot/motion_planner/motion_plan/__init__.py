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
import numpy as np

from dynamic_graph import plug

from dynamic_graph.sot.motion_planner.feet_follower_graph_with_correction \
    import FeetFollowerGraphWithCorrection

from dynamic_graph.sot.motion_planner.math import *

from dynamic_graph.corba_server import CorbaServer

from dynamic_graph.sot.motion_planner.motion_plan.control import *
from dynamic_graph.sot.motion_planner.motion_plan.environment import *
from dynamic_graph.sot.motion_planner.motion_plan.error_strategy import *
from dynamic_graph.sot.motion_planner.motion_plan.motion import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *



class MotionPlan(object):
    robot = None
    solver = None

    feetFollower = None

    plan = None
    duration = 0
    motion = []
    control = []
    footsteps = []
    environment = {}

    trace = None

    maxX = FeetFollowerGraphWithCorrection.maxX
    maxY = FeetFollowerGraphWithCorrection.maxY
    maxTheta = FeetFollowerGraphWithCorrection.maxTheta

    def __init__(self, filename, robot, solver, logger):
        self.robot = robot
        self.solver = solver
        self.logger = logger

        self.logger.info('loading motion plan file \'{0}\''.format(filename))
        self.plan = yaml.load(open(filename, "r"))

        self.duration = float(self.plan['duration'])

        # Create CORBA server if required.
        self.corba = CorbaServer('corba_server')

        self.logger.debug('loading environment')
        self.loadEnvironment()
        self.logger.debug('loading motion elements')
        self.loadMotion()
        self.logger.debug('loading control elements')
        self.loadControl()

        # For now, only 1 feet follower is allowed (must start at t=0).
        feetFollower = find(lambda e: type(e) == MotionWalk, self.motion)
        hasControl = len(self.control) > 0

        if hasControl and feetFollower:
            self.feetFollower = FeetFollowerGraphWithCorrection(
                robot, solver, self.motion[0].feetFollower,
                MotionPlanErrorEstimationStrategy,
                maxX = self.maxX, maxY = self.maxY,
                maxTheta = self.maxTheta)
            self.feetFollower.errorEstimationStrategy.motionPlan = self
                #FIXME: not enough generic
            self.feetFollower.feetFollower.setFootsteps(
                2., makeFootsteps(self.footsteps))
        elif feetFollower:
            self.feetFollower = feetFollower.feetFollower
        else:
            self.feetFollower = None

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

        motionClasses = [MotionWalk, MotionTask]

        for motion in self.plan['motion']:
            if len(motion.items()) != 1:
                raise RuntimeError('each motion should have only one type')
            (tag, data) = motion.items()[0]
            cls = find(lambda c: c.yaml_tag == tag, motionClasses)

            if not cls:
                raise RuntimeError('invalid motion element')
            self.motion.append(cls(self, data))
            self.logger.debug('adding motion element \'{0}\''.format(tag))


    def loadControl(self):
        if not 'control' in self.plan or not self.plan['control']:
            return

        controlClasses = [ControlConstant, ControlMocap, ControlViSP,
                          ControlHueblob, ControlVirtualSensor]

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
            res += '\t* {0}\n'.format(str(motion))
        res += '\n'
        res += 'Control:\n'
        res += '--------\n'
        for control in self.control:
            res += '\t* {0}\n'.format(str(control))
        res += '\n'
        res += str(self.feetFollower)
        res += '\n'
        return res

    def start(self):
        self.logger.info('execution starts')
        if self.feetFollower:
            self.feetFollower.start()

    def canStart(self):
        if self.feetFollower:
            return self.feetFollower.canStart()
        else:
            return True
