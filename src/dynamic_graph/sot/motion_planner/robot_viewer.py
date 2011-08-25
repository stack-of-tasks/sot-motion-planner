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
from dynamic_graph.sot.motion_planner.math import *

def createObject(clt, name, obj, elements):
    if not clt:
        return
    if not name in elements:
        clt.createElement('object', name,
                          os.environ['HOME'] + '/.robotviewer/' + obj + '.py')
        clt.enableElement(name)

def drawFootsteps(clt, plan, robot, startLeft, startRight, elements,
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
    pos = np.matrix(startRight, dtype=np.float)
    pos[2,3] = 0.

    for step in footsteps:
        pos = pos * XYThetaToHomogeneousMatrix(
            (step['x'], step['y'], step['theta']))
        name = 'step_' + str(i)
        if create:
            model = 'left-footstep'
            if i % 2 == 1:
                model = 'right-footstep'
            createObject(clt, name, model, elements)
        clt.updateElementConfig(name, pose(pos))
        i = i + 1

def drawFootstepsFromFile(clt, filename,  startRight, elements, suffix='',
                          create = True):
    f = open('/tmp/' + str(filename))

    footsteps = []
    for line in f:
        values = line.split()
        footsteps.append({'x': float(values[0]),
                          'y': float(values[1]),
                          'theta': float(values[2])})

    i = 0
    pos = np.matrix(startRight, dtype=np.float)
    pos[2,3] = 0.

    for step in footsteps:
        pos = pos * XYThetaToHomogeneousMatrix(
            (step['x'], step['y'], step['theta']))
        name = 'step_' + str(i) + str(suffix)
        if create:
            model = 'left-footstep'
            if i % 2 == 1:
                model = 'right-footstep'
            createObject(clt, name, model, elements)
        clt.updateElementConfig(name, pose(pos))
        i = i + 1


def drawObstacles(clt, plan, robot, elements):
    i = 0
    for control in plan.control:
        if control[0] == 'virtual-sensor':
            namePlanned = 'obstaclePlanned' + str(i)
            nameReal = 'obstacleReal' + str(i)
            createObject(clt, namePlanned, 'disk', elements)
            createObject(clt, nameReal, 'disk2', elements)

            posPlanned = control[2].expectedObstaclePosition.value
            posReal = control[2].obstaclePosition.value

            clt.updateElementConfig(namePlanned,
            [posPlanned[0][3], posPlanned[1][3], posPlanned[2][3], 0, 0, 0])
            clt.updateElementConfig(nameReal,
            [posReal[0][3], posReal[1][3], posReal[2][3], 0, 0, 0])

            i = i + 1
