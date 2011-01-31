#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
#
# This file is part of sot-motion-planner.
# sot-motion-planner is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-motion-planner is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

from __future__ import division
from sympy import *
from sympy.matrices import *

r = Symbol('r')

x = Symbol('x')
y = Symbol('y')
theta = Symbol('theta')

xr = Symbol('x_0')
yr = Symbol('y_0')

sensorOffset = Symbol('S')

S = Matrix([x + r * sin(sensorOffset + theta), y - r * cos(sensorOffset + theta)])

Q = Matrix([x, y, theta])

print "dS/dQ:"
pprint(S.jacobian(Q))


P = Matrix([.5 * (xr - x) * (xr - x) + .5 * (yr - y) * (yr - y)])
S = Matrix([x, y])

print "dP/dS:"
pprint(P.jacobian(S))
