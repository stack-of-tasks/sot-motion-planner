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

import numpy as np
from math import acos, cos, sin, pi

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import Localizer

from dynamic_graph.sot.dynamics.hrp2 import Hrp2

# Vector3
def makeVector3(x = 0., y = 0., z = 0.):
    return np.array([x, y, z], dtype=np.dtype(np.float))

# Rotation matrices
def makeRotationMatrix():
    return np.asmatrix(np.identity(3, dtype=np.dtype(np.float)))

def hat(a):
    return np.matrix(
        [[ 0,    -a[2],  a[1]],
         [ a[2],  0,    -a[0]],
         [-a[1],  a[0],  0   ]],
        dtype = np.dtype(np.float)
        )

# Homogeneous matrices
def makeHomogeneousMatrix(R = None, t = None):
    res = np.asmatrix(np.identity(4, dtype=np.dtype(np.float)))
    if R:
        res[0:3,0:3] = R
    if t:
        res[0:3,3:] = map(lambda x: [x], t)
    return res

def getR(H):
    return H[0:3,0:3]
def getT(H):
    return H[0:3,3:].transpose()[0]

# rotation vector representation
def makeRotationVector(x = 0., y = 0., z = 0.):
    return makeVector3(x, y, z)

def rotationVectorToRotationMAtrix(rotationVector):
    norm = np.linalg.norm(rotationVector)
    norm_square = norm * norm
    sin_norm = sin(norm)
    one_minus_cos_norm = 1 - cos(norm)
    hat_ = hat(rotationVector)
    hat_square = hat_ * hat_

    m = makeRotationMatrix() # returns identity.
    m += hat_ / norm * sin_norm
    m += hat_square / norm_square * one_minus_cos_norm
    return m

def rotationMatrixToRotationVector(rotationMatrix):
    theta = acos (.5 * (rotationMatrix.trace()[0,0] - 1))
    v = makeRotationVector()
    v[0] = rotationMatrix[2,1] - rotationMatrix[1,2]
    v[1] = rotationMatrix[0,2] - rotationMatrix[2,0]
    v[2] = rotationMatrix[1,0] - rotationMatrix[0,1]
    v *= theta / (2. * sin(theta))
    return v


###################################
checkAlgebra = False
if checkAlgebra:
    V1 = makeVector3(1., 2., 3.)
    hatV1 = hat(V1)

    H1 = makeHomogeneousMatrix()
    H2 = makeHomogeneousMatrix(t = [1., 2., 3.])
    H3 = makeHomogeneousMatrix(R = [[1., 2., 3.],
                                    [4., 5., 6.],
                                    [7., 8., 9.]])

    print "V1"
    print V1
    print "hatV1"
    print hatV1
    print "hatV1 . V1"
    print np.inner(hatV1,V1)
    print "H1"
    print H1
    print "H2"
    print H2
    print "H3"
    print H3

    print getR(H1)
    print getT(H2)

    print rotationVectorToRotationMAtrix(makeRotationVector(0., 0., 1.))
    print rotationVectorToRotationMAtrix(makeRotationVector(0., 0., 2.))
    print rotationVectorToRotationMAtrix(makeRotationVector(0., 1., 0.))
    print rotationVectorToRotationMAtrix(makeRotationVector(1., 0., 0.))
    print rotationVectorToRotationMAtrix(makeRotationVector(1., 1., 0.))
    print rotationVectorToRotationMAtrix(makeRotationVector(0., 1., 1.))
    print rotationVectorToRotationMAtrix(makeRotationVector(1., 0., 1.))
    print rotationVectorToRotationMAtrix(makeRotationVector(1., 1., 1.))

    print rotationMatrixToRotationVector( \
        rotationVectorToRotationMAtrix(makeRotationVector(1., 1., 1.)))
    exit()

# Robot is HRP-2 robot.
#
# gaze operational point gives the camera optical frame
#
#
#
#
#
#

robot = Hrp2("robot", True)
robot.dynamic.createOpPoint('gaze', 'gaze')
robot.dynamic.gaze.recompute(0)
robot.dynamic.Jgaze.recompute(0)

l = Localizer('localizer')

q = np.array([0., 0. ,0.])
error = np.array([1., 0. , 0.])

expectedRobot = np.array([0., 0., 0.]) # x, y, theta
realRobot = np.array(map(lambda (p, e): p + e, zip(expectedRobot, error)))

f = 1.
(px, py) = (1., 1.)
(u0, v0) = (0., 0.)
C = np.matrix(
    [[ f * px, 0.,      u0 ],
     [ 0.,     -f * py, v0 ],
     [ 0.,     0.,      1. ]],
    dtype = np.dtype(np.float)
    )

def S(q, sensorId):
    return robot.dynamic.gaze.value

def P(sensorPosition, landmark):
    _3to4 = np.matrix(
        [[ 1., 0., 0., 0.],
         [ 0., 1., 0., 0.],
         [ 0., 0., 1., 0.]],
        dtype = np.dtype(np.float)
        )
    referencePoint = np.array([1., 2., 3., 1.], dtype=np.float)

    referencePoint_ = np.inner(sensorPosition, referencePoint)
    tmp = np.inner(C * _3to4, referencePoint_)
    return np.array([tmp[0]/tmp[2], tmp[1]/tmp[2]], dtype=np.float)

def dS(q, sensorId):
    return robot.dynamic.Jgaze.value #FIXME:

def dP(sensorPosition, landmark):
    pass #FIXME:


###############################

print P(np.matrix(robot.dynamic.gaze.value), 0)
