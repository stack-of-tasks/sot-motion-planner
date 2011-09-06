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
import os
from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan import *

def createObject(clt, name, obj, elements):
    if not clt:
        return
    if not name in elements:
        clt.createElement('object', name,
                          os.environ['HOME'] + '/.robotviewer/' + obj)
        elements.append(name)
        clt.enableElement(name)

def drawFootsteps(clt, plan, robot, startLeft, startRight, elements,
                  create = True, filename = None):
    if not plan.feetFollower:
        return


    footsteps = [{'x': 0., 'y': 0., 'theta': 0.},
                 {'x': 0., 'y': +0.19, 'theta': 0.}]

    try:
        footstepsSignal = plan.feetFollower.feetFollower.dbgFootsteps
        footstepsSignal.recompute(footstepsSignal.time + 1)

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
    except AttributeError:
        # If correction is not used, just print footsteps
        # from the plan.
        footsteps += plan.footsteps

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
            model = 'left-footstep.py'
            if i % 2 == 1:
                model = 'right-footstep.py'
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
        if type(control) != ControlVirtualSensor:
            continue
        namePlanned = 'obstaclePlanned' + str(i)
        nameReal = 'obstacleReal' + str(i)
        obj = plan.environment[control.objectName]

        filenameFmt = '{0}/.robotviewer/{1}'
        filename = filenameFmt.format(os.environ['HOME'],
                                      obj.plannedModel)
        filenameEstimated = filenameFmt.format(os.environ['HOME'],
                                               obj.estimatedModel)

        createObject(clt, namePlanned, obj.plannedModel, elements)
        createObject(clt, nameReal, obj.estimatedModel, elements)

        posPlanned = np.matrix(control.virtualSensor.expectedObstaclePosition.value)
        posReal = np.matrix(control.virtualSensor.obstaclePosition.value)

        clt.updateElementConfig(namePlanned,
                                Pose6d.fromRotationMatrix(posPlanned).pose())
        clt.updateElementConfig(nameReal,
                                Pose6d.fromRotationMatrix(posReal).pose())
        i = i + 1
