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
from math import acos, atan2, cos, sin, pi, sqrt

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import Localizer

from dynamic_graph.sot.dynamics.hrp2 import Hrp2

try:
    from dynamic_graph.sot.core import OpPointModifior
    OpPointModifier = OpPointModifior
except ImportError:
    from dynamic_graph.sot.core import OpPointModifier


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
    return np.array([H[0,3], H[1,3], H[2,3]], dtype=np.float)

def inverseHomogeneousMatrix(H):
    print H
    H[0:3,0:3] = np.linalg.inv(H[0:3,0:3])
    H[0,3] *= -1.; H[1,3] *= -1.; H[2,3] *= -1.
    return H

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

# roll, pitch, yaw
def roll(rotationMatrix):
    return atan2(rotationMatrix[1,0], rotationMatrix[0,0])

def pitch(rotationMatrix):
    return atan2(-rotationMatrix[2,0],
                  sqrt(rotationMatrix[2,1]**2 + rotationMatrix[2,2]**2))

def yaw(rotationMatrix):
    return atan2(rotationMatrix[2,1], rotationMatrix[2,2])

def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

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
# q = (x, y, theta)
#
# S(q) = (x, y, z?, u, v, w)
# P(S(q)) = (x, y)
#
#               dS/dx dS/dy dS/dz dS/du dS/dv dS/dw
# dP(s(q)) = [[                                     ] x
#             [                                     ] y
#
#
# P = C * [[1,0,0,0] * e^s(q) * [[x_ref]
#          [0,1,0,0]             [y_ref]
#          [0,0,1,0]]            [z_ref]]
#
#
# P = C * [[1,0,0,0]
#          [0,1,0,0]
#          [0,0,1,0]]
#     * (I
#        + s(q)^ / |s(q)| * sin(|s(q)|)
#        + (s(q)^)^2 / |s(q)|^2 * (1 - cos(|s(q)|)))
#     * [[x_ref]
#        [y_ref]]
#        [z_ref]]
#
#
#

robot = Hrp2("robot", True)
robot.dynamic.gaze.recompute(0)
robot.dynamic.Jgaze.recompute(0)

# Additional frames for sensors.
lwcam = OpPointModifier('lwcam')
plug(robot.dynamic.gaze, lwcam.positionIN)
plug(robot.dynamic.Jgaze, lwcam.jacobianIN)

# HRP2-14 extrinsic camera parameters
leftWideCamera = ((1., 0., 0., 0.035),
                  (0., 1., 0., 0.072),
                  (0., 0., 1., 0.075),
                  (0., 0., 0., 1.))
lwcam.setTransformation(leftWideCamera)
lwcam.position.recompute(0)
lwcam.jacobian.recompute(0)


# Localization
l = Localizer('localizer')

q = np.array([0., 0. ,0.])
error = np.array([1., 0. , 0.])

expectedRobot = np.array([0., 0., 0.]) # x, y, theta
realRobot = np.array(map(lambda (p, e): p + e, zip(expectedRobot, error)))

sensors = [lwcam]

f = 1.
(px, py) = (1., 1.)
(u0, v0) = (0., 0.)
C = np.matrix(
    [[ f * px, 0.,     u0 ],
     [ 0.,     f * py, v0 ],
     [ 0.,     0.,     1. ]],
    dtype = np.dtype(np.float)
    )

def S(q, sensorId):
    #FIXME: use q
    return np.matrix(sensors[sensorId].position.value, dtype=np.float)
def dS(q, sensorId):
    #FIXME: use q
    return np.matrix(sensors[sensorId].jacobian.value, dtype=np.float)[0:6,0:6]

def P(sensorPosition, referencePoint):
    #FIXME: replace referencePoint by landmark

    # This rotates the world frame to the camera frame.
    # 3d frames are front/left/up (FLU).
    # 2d images coordinates are (0,0) on the top-left corner
    # x, y increases toward bottom-right corner.
    _3to4 = np.matrix(
        [[ 0., -1.,  0., 0.],
         [ 0.,  0., -1., 0.],
         [ 1.,  0.,  0., 0.]],
        dtype = np.dtype(np.float)
        )

    referencePoint_ = np.inner(inverseHomogeneousMatrix(sensorPosition),
                               referencePoint)

    print referencePoint_

    tmp = np.inner(C * _3to4, referencePoint_)

    print tmp[0]
    print tmp[1]
    print tmp[2]

    return np.array([tmp[0]/tmp[2], tmp[1]/tmp[2]], dtype=np.float)


def dP(sensorPosition, landmark):
    t = getT(sensorPosition)
    return np.matrix(
        [[ (-t[0] * px) / (t[2] * t[2]), -1.,  0., 0., 0., 0.],
         [ (-t[1] * py) / (t[2] * t[2]),  0., -1., 0., 0., 0.]],
        dtype = np.dtype(np.float)
        )


###############################

print "S(q)"
print S(0,0)
print "dS(q)/dq (position + rotation vector)^2"
print dS(0,0)

print "P"
print P(S(0,0), np.array([1., 0.072, 0.723, 1.], dtype=np.float))

print "dP"
print dP(S(0, 0), 0)


# Localizer setup.
l.add_landmark_observation('obs')

l.obs_JfeatureReferencePosition.value = matrixToTuple(dP(S(0, 0), 0))
l.obs_JsensorPosition.value = matrixToTuple(dS(0, 0))
l.obs_weight.value = (1., 1.)

l.obs_featureObservedPosition.value = (0., 0.)
l.obs_featureReferencePosition.value = (0., 0.)

l.configurationOffset.recompute(0)

print "Offset:"
print l.configurationOffset.value

#FIXME: W has 6 columns , localizer expects 3.
# Handle properly estimation vs observations of dofs in C++
#FIXME: implement angular velocities in dP.
