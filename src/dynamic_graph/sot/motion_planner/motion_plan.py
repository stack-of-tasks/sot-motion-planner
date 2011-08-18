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

from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph
from dynamic_graph.sot.motion_planner.feet_follower_graph_with_correction \
    import FeetFollowerGraphWithCorrection

from dynamic_graph.sot.motion_planner.error_estimation_strategy \
    import MotionCaptureErrorEstimationStrategy


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


class MotionPlan(object):
    robot = None
    solver = None

    plan = None
    duration = 0
    motion = []
    control = []

    def __init__(self, filename, robot, solver):
        self.robot = robot
        self.solver = solver
        self.plan = yaml.load(open(filename, "r"))
        self.duration = self.plan['duration']

        self.loadMotion()
        self.loadControl()

        if len(self.motion):
            if len(self.control):
                # For now, do not handle multiple trackers.
                self.feetFollower = \
                    FeetFollowerGraphWithCorrection(
                    robot, solver, self.motion[0][1], self.control[0][1])
            else:
                self.feetFollower = self.motion[0][1]
        else:
            self.feetFollower = None

    def loadMotionWalk(self, motionWalk):
        steps = convertToNPFootstepsStack(motionWalk['footsteps'])
        feetFollower = FeetFollowerAnalyticalPgGraph(
            self.robot, self.solver, steps)
        self.motion.append([motionWalk, feetFollower])
        print("successfully loaded walk motion")

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

        for motion in self.plan['motion']:
            if 'walk' in motion:
                self.loadMotionWalk(motion['walk'])
            elif 'task' in motion:
                self.loadMotionTask(motion['task'])
            else:
                raise "invalid motion element"

    def loadControlMocap(self, mocap):
        self.control.append([mocap, MotionCaptureErrorEstimationStrategy])
        print("successfully loaded motion capture control")

    def loadControlVisp(self, visp):
        #self.control.append([FIXME])
        raise NotImplementedError

    def loadControlHueblob(self, hueblob):
        #self.control.append([FIXME])
        raise NotImplementedError

    def loadControlVirtualSensor(self, virtualSensor):
        #self.control.append([FIXME])
        pass #FIXME: TODO


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
            print('\t* ' + str(control[1]))
        print('')
        print('')
