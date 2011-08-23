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

import os

os.system("rm /tmp/feet_follower_*.dat")

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner import *

logWaist = False
def logWaistTrajectory(robot, f):
    for i in range(4):
        for j in range(4):
            f.write(str(robot.dynamic.waist.value[i][j]) + ' ')
    f.write('\n')


(options, args) = parser.parse_args()

if not len(args):
    raise RuntimeError("motion plan needed")

motionPlan = MotionPlan(args[0], robot, solver)

def createObject(clt, name, obj):
    if not clt:
        return
    elements = clt.listElement()
    if name in elements:
        clt.destroyElement(name)
    clt.createElement('object', name,
                      os.environ['HOME'] + '/.robotviewer/' + obj + '.py')
    clt.enableElement(name)

def drawFootsteps(clt, plan, robot, startLeft, startRight,
                  create=True, filename=None):
    if not plan.feetFollower:
        return

    footstepsSignal = plan.feetFollower.feetFollower.dbgFootsteps
    footstepsSignal.recompute(footstepsSignal.time + 1)

    footsteps = [{'x': 0., 'y': 0., 'theta': 0.},
                 {'x': 0., 'y': +0.19, 'theta': 0.}]
    i = 0
    stepX = 0
    stepY = 0
    stepTheta = 0
    for step in footstepsSignal.value:
        if i == 0:
            stepX = step
        elif i == 1:
            stepY = step
        elif i == 2:
            stepTheta = step
        else:
            raise RuntimeError

        if i == 2:
            i = 0
            footsteps.append({'x': stepX,
                              'y': stepY,
                              'theta': stepTheta})
        else:
            i = i + 1

    if filename:
        f = open('/tmp/' + str(filename), 'w')
        for step in footsteps:
            f.write(str(step['x'])
                    + ' ' + str(step['y'])
                    + ' ' + str(step['theta'])
                    + '\n')

    i = 0
    x = startRight[0]
    y = startRight[1]

    for step in footsteps:
        x += step['x']
        y += step['y']
        name = 'step_' + str(i)
        if create:
            model = 'left-footstep'
            if i % 2 == 1:
                model = 'right-footstep'
            createObject(clt, name, model)
        clt.updateElementConfig(
            name, [x, y, 0, 0, 0, 0])
        i = i + 1

def drawFootstepsFromFile(clt, filename,  startRight, suffix='',
                          create = True):
    f = open('/tmp/' + str(filename))

    footsteps = []
    for line in f:
        values = line.split()
        footsteps.append({'x': float(values[0]),
                          'y': float(values[1]),
                          'theta': float(values[2])})

    i = 0
    x = startRight[0]
    y = startRight[1]

    for step in footsteps:
        x += step['x']
        y += step['y']
        name = 'step_' + str(i) + str(suffix)
        if create:
            model = 'left-footstep'
            if i % 2 == 1:
                model = 'right-footstep'
            createObject(clt, name, model)
        clt.updateElementConfig(
            name, [x, y, 0, 0, 0, 0])
        i = i + 1


def drawObstacles(clt, plan, robot):
    i = 0
    for control in plan.control:
        if control[0] == 'virtual-sensor':
            namePlanned = 'obstaclePlanned' + str(i)
            nameReal = 'obstacleReal' + str(i)
            createObject(clt, namePlanned, 'disk')
            createObject(clt, nameReal, 'disk2')

            posPlanned = control[2].expectedObstaclePosition.value
            posReal = control[2].obstaclePosition.value

            clt.updateElementConfig(namePlanned,
            [posPlanned[0][3], posPlanned[1][3], posPlanned[2][3], 0, 0, 0])
            clt.updateElementConfig(nameReal,
            [posReal[0][3], posReal[1][3], posReal[2][3], 0, 0, 0])

            i = i + 1


def play(plan, afterStart = None):
    global logWaist

    maxIter = int(plan.duration / 0.005)
    print maxIter

    while not plan.canStart():
        robot.device.increment(timeStep)
    plan.start()
    print("start")
    if afterStart:
        afterStart()
    print("started")

    t = 0
    startLeft = (robot.dynamic.signal('left-ankle').value[0][3],
                 robot.dynamic.signal('left-ankle').value[1][3])
    startRight = (robot.dynamic.signal('right-ankle').value[0][3],
                  robot.dynamic.signal('right-ankle').value[1][3])

    # Main.
    #  Main loop
    #logCfg = open("/tmp/cfg.dat", "w")
    f = open("/tmp/waist.dat", "w")

    if clt:
        elements = clt.listElement()
        if not 'hrp' in elements:
            raise RuntimeError
        drawFootsteps(clt, plan, robot, startLeft, startRight,
                      filename='footsteps-orig.dat')
        drawObstacles(clt, plan, robot)

        #drawFootstepsFromFile(clt, 'toto.dat', startRight, 'final')

    for i in xrange(maxIter):
        robot.device.increment(timeStep)

        #log(logCfg)
        if clt:
            drawFootsteps(clt, plan, robot, startLeft, startRight, False)
            clt.updateElementConfig(
                'hrp', robot.smallToFull(robot.device.state.value))

        if logWaist:
            logWaistTrajectory(robot, f)


    if plan.feetFollower:
        plan.feetFollower.trace.dump()
    if clt:
        drawFootsteps(clt, plan, robot, startLeft, startRight, False,
                      filename='footsteps-final.dat')

motionPlan.displayMotion()
play(motionPlan)
