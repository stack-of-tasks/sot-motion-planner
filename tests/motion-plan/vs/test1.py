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

from dynamic_graph.sot.motion_planner import \
    ErrorEstimator, ErrorMerger, VirtualSensor

vs = VirtualSensor('vs')

def test(vs, position, estimated, planned):
    vs.expectedObstaclePosition.value = (
        (1., 0., 0., planned[0]),
        (0., 1., 0., planned[1]),
        (0., 0., 1., planned[2]),
        (0., 0., 0., 1.), )
    
    vs.obstaclePosition.value = (
        (1., 0., 0., estimated[0]),
        (0., 1., 0., estimated[1]),
        (0., 0., 1., estimated[2]),
        (0., 0., 0., 1.), )

    vs.planned.value = (
        (1., 0., 0., position[0]),
        (0., 1., 0., position[1]),
        (0., 0., 1., position[2]),
        (0., 0., 0., 1.), )

    vs.position.recompute(vs.position.time + 1)
    vs.positionTimestamp.recompute(vs.positionTimestamp.time + 1)

    print("---")
    print("planned robot pos     = " + str(position))
    print("estimated feature pos = " + str(estimated))
    print("planned feature pos   = " + str(planned))
    print("position              = " + str(vs.position.value))
    print("positionTimestamp     = " + str(vs.positionTimestamp.value))

    return vs.position.value

print test(vs, [0., 0., 0.], [0., 0., 0.], [0., 0., 0.]) == (0., 0., 0.)
print test(vs, [1., 2., 0.], [0., 0., 0.], [0., 0., 0.]) == (1., 2., 0.)

print test(vs, [5., 4., 0.], [1., 0., 0.], [2., 0., 0.]) == (6., 4., 0.)
