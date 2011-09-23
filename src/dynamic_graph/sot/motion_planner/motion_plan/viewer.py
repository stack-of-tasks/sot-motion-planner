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
import os, signal, sys, time

from dynamic_graph.sot.motion_planner.robot_viewer import *
from dynamic_graph.sot.motion_planner.motion_plan.control import *

class TextColor(object):
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    @staticmethod
    def toColor(string, color):
        return color + string + TextColor.ENDC

    @staticmethod
    def write(stream, string, color):
        stream.write(TextColor.toColor(string, color))

class WaitingAnimation(object):
    currentPosition = None
    sequence = ['|', '/', '-', '\\', '-', '/']

    def write(self, stream):
        if self.currentPosition == None:
            self.currentPosition = 0
            stream.write(' ')
        stream.write('\b' + self.sequence[self.currentPosition])
        self.currentPosition += 1
        if self.currentPosition >= len(self.sequence):
            self.currentPosition = 0

class MotionPlanViewer(object):
    robot = None
    plan = None
    client = None
    enableObstacles = True
    enableFootsteps = True
    enableRobot = True
    shouldExit = False
    elements = None
    initialAnklePositions = None

    step = 5e-3
    robotElementName = 'hrp'

    storedOpPoints = ['waist', 'gaze', 'zmp']

    def __init__(self, plan, robot, client,
                 logger,
                 enableObstacles = True,
                 enableFootsteps = True,
                 enableRobot = True,
                 logCfg = True,
                 logOpPoints = False):
        logger.debug('creating MotionPlanViewer instance')
        self.robot = robot
        self.plan = plan
        self.client = client
        self.logger = logger
        self.enableObstacles = enableObstacles
        self.enableFootsteps = enableFootsteps
        self.enableRobot = enableRobot
        self.elements = client.listElements()
        self.logCfg = logCfg
        self.logOpPoints = logOpPoints

        # Store operational points trajectories.
        self.positions = {'waist': [], 'gaze': [], 'zmp': []}
        self.configurations = []

        #FIXME: does not work for now, ""race condition"" between the GL/CORBA
        # thread in robot-viewer.
        #self.cleanObjects()

        self.initialAnklePositions = (
            self.robot.dynamic.signal('left-ankle').value,
            self.robot.dynamic.signal('right-ankle').value
            )

        if not self.robotElementName in self.elements:
            raise RuntimeError(
                'robot \'{0}\' does not exist in robot-viewer'.format(
                    self.robotElementName))

        #FIXME: find a better way to wipe out traces.
        os.system("rm -f /tmp/feet_follower_*.dat")

        self.loadEnvironment()

    def handler(self, signum, frame):
        self.shouldExit = True
        self.reset()

    def cleanObjects(self):
        for obj in self.elements:
            if obj != self.robotElementName:
                self.client.destroyElement(obj)


    def createObject(self, name, filename, cfg = None):
        if not name in self.elements:
            filename = searchFile(filename, self.plan.defaultDirectories)
            self.client.createElement('object', name, filename)
            self.client.enableElement(name)
            self.elements.append(name)
        if cfg:
            self.client.updateElementConfig(name, cfg)

    def loadEnvironment(self):
        for (name, obj) in self.plan.environment.items():
            namePlanned = name + '-planned'
            nameEstimated = name + '-estimated'
            filename = obj.plannedModel

            self.createObject(namePlanned, filename, obj.plannedPosition.pose(
                    rotation = False))

            #FIXME: not enough generic, should probably not be here.
            for control in self.plan.control:
                if type(control) != ControlVirtualSensor:
                    continue
                if control.objectName != name:
                    continue
                filename = obj.estimatedModel
                self.createObject(nameEstimated, filename,
                                  control.position.pose(rotation = False))



    def updateRobot(self, cfg = None):
        if not cfg:
            cfg = self.robot.smallToFull(self.robot.device.state.value)
        self.client.updateElementConfig(self.robotElementName, cfg)

    def update(self):
        self.updateRobot()
        if self.enableFootsteps:
            drawFootsteps(self.client, self.plan, self.robot,
                          self.initialAnklePositions[0],
                          self.initialAnklePositions[1],
                          self.elements, create = False)
        if self.enableObstacles:
            drawObstacles(self.client, self.plan, self.robot, self.elements)

    def storePositions(self):
        if self.logOpPoints:
            for op in self.storedOpPoints:
                f = open('/tmp/{0}.dat'.format(op), 'w+')
                for w in self.positions[op]:
                    w = np.array(w)
                    for e in w.flat:
                        f.write('{0} '.format(e))
                    f.write('\n')
            self.logger.info('saving op points trajectories')

        if self.logCfg:
            f = open('/tmp/movement.pos', 'w+')
            t = 0.
            for cfg in self.configurations:
                cfg = np.array(cfg)
                t += 0.005
                f.write('{0} '.format(t))
                for e in cfg.flat:
                    f.write('{0} '.format(e))
                f.write('\n')
            self.logger.info('saving configurations')

    def reset(self):
        self.logger.info('execution interrupted')

        # Write traces.
        if self.plan.feetFollower:
            self.plan.feetFollower.trace.dump()

        self.storePositions()

        # Try to reset the robot.
        try:
            self.updateRobot(self.robot.smallToFull(self.robot.halfSitting))
        except:
            print('failed to reset the robot')

    def play(self):
        handle = lambda x, y: self.handler(x, y)
        for s in [signal.SIGTERM, signal.SIGINT, signal.SIGABRT]:
            signal.signal(s, handle)

        nIterations = int(self.plan.duration / self.step)

        if self.enableFootsteps:
            drawFootsteps(self.client, self.plan, self.robot,
                          self.initialAnklePositions[0],
                          self.initialAnklePositions[1],
                          self.elements, create = True)

        animation = WaitingAnimation()
        sys.stdout.write('Waiting for motion to initialize... ')
        while not self.plan.canStart() and not self.shouldExit:
            self.robot.device.increment(self.step)
            animation.write(sys.stdout)
            sys.stdout.flush()
        sys.stdout.write('\n')
        if self.shouldExit:
            return

        self.plan.start()
        self.logger.info('execution started')

        previousCfg = None

        fmt = 'Playing... {0:>4d}/{1:<4d} ({2:>4d}ms, {3:>4d}ms)'
        discontinuityFmt = \
            '/!\\ discontinuity detected! invalid joint {0:<2d} /!\\'
        for n in xrange(nIterations + 1):
            if self.shouldExit:
                sys.stdout.write('\n')
                return

            startTime = time.clock()
            self.update()
            controlStartTime = time.clock()
            self.robot.device.increment(self.step)
            endTime = time.clock()

            for op in self.storedOpPoints:
                self.robot.dynamic.signal(op).recompute(
                    self.robot.dynamic.signal(op).time + 1)
                self.positions[op].append(self.robot.dynamic.signal(op).value)
            self.configurations.append(
                self.robot.smallToFull(self.robot.device.state.value)[6:])

            # Safety checks.
            cfg = self.robot.device.state.value
            if previousCfg:
                d = np.array(cfg)-np.array(previousCfg)
                for i in xrange(len(d)):
                    if abs(d[i]) > 0.1:
                        sys.stdout.write(discontinuityFmt.format(i)+'\n')
            previousCfg = cfg

            # Output.
            tControl = endTime - controlStartTime
            tAll = endTime - startTime

            s = fmt.format(n, nIterations,
                           int(tControl * 1000.), int(tAll * 1000.))
            self.logger.debug(s)

            if tControl >= self.step:
                s = TextColor.toColor('\r' + s, TextColor.FAIL)
            elif tAll >= self.step:
                s = TextColor.toColor('\r' + s, TextColor.WARNING)
            else:
                s = TextColor.toColor('\r' + s, TextColor.OKGREEN)
            sys.stdout.write(s)
            sys.stdout.flush()

            if tAll < self.step:
                time.sleep(self.step - tAll)
        sys.stdout.write('\n')
        self.logger.info('execution finished')
        self.storePositions()
        if self.plan.feetFollower:
            self.plan.feetFollower.trace.dump()
