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

def addTrace(robot, trace, entityName, signalName):
    trace.add(entityName + '.' + signalName,
              entityName + '-' + signalName)
    robot.device.after.addSignal(entityName + '.' + signalName)

def convertToNPFootstepsStack(footsteps):
    minSlides = (-1.52, -0.76)

    res = []
    for step in footsteps:
        slide1 = step.get('slide1')
        slide2 = step.get('slide2')

        if not slide1:
            if len(res) == 0:
                slide1 = 0.
            else:
                slide1 = minSlides[0]
        if not slide2:
            slide2 = minSlides[1]
        res.append((slide1,
                    step.get('horizontal-distance', 0.24),
                    step.get('height', 0.25),
                    slide2,
                    step.get('x', 0.), step.get('y', 0.),
                    step.get('theta', 0.)))
    return tuple(res)

def makeFootsteps(footsteps):
    res = []
    for step in footsteps:
        res.append(step.get('x', 0.))
        res.append(step.get('y', 0.))
        res.append(step.get('theta', 0.))
    return tuple(res)

def find(f, seq):
  """Return first item in sequence where f(item) == True."""
  for item in seq:
    if f(item):
      return item

def checkDict(k, d):
    if not k in d:
        raise RuntimError('missing key {0}'.format(k))

def searchFile(f, defaultDirectories):
    for e in [''] + defaultDirectories:
        try:
            filename = '{0}/{1}'.format(e, f)
            open(filename, "r")
            return filename
        except IOError:
            pass
    raise RuntimeError('failed to find file \'{0}\' in {1}'.format(
            f, defaultDirectories))
