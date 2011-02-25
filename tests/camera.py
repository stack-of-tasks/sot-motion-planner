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

from __future__ import print_function
import numpy as np
from math import acos, atan2, cos, sin, pi, sqrt

from dynamic_graph import plug
from dynamic_graph.sot.motion_planner import Localizer

from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas

try:
    from dynamic_graph.sot.core import OpPointModifior
    OpPointModifier = OpPointModifior
except ImportError:
    from dynamic_graph.sot.core import OpPointModifier

from dynamic_graph.sot.core import FeatureVisualPoint


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
    if not R is None:
        res[0:3,0:3] = R
    if not t is None:
        res[0:3,3:] = map(lambda x: [x], t)
    return res

def getR(H):
    return H[0:3,0:3]
def getT(H):
    return np.array([H[0,3], H[1,3], H[2,3]], dtype=np.float)

def inverseHomogeneousMatrix(H):
    return np.linalg.inv(H)

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

    print("V1")
    print(V1)
    print("hatV1")
    print(hatV1)
    print("hatV1 . V1")
    print(np.inner(hatV1,V1))
    print("H1")
    print(H1)
    print("H2")
    print(H2)
    print("H3")
    print(H3)

    print(getR(H1))
    print(getT(H2))

    print(rotationVectorToRotationMAtrix(makeRotationVector(0., 0., 1.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(0., 0., 2.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(0., 1., 0.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(1., 0., 0.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(1., 1., 0.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(0., 1., 1.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(1., 0., 1.)))
    print(rotationVectorToRotationMAtrix(makeRotationVector(1., 1., 1.)))

    print(rotationMatrixToRotationVector( \
        rotationVectorToRotationMAtrix(makeRotationVector(1., 1., 1.))))
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

robot = Hrp2Laas("robot")


# Camera related frames.

w_M_c = OpPointModifier('w_M_c')

# Compute the transformation between the
# gaze and the left bottom (wide) camera.

# Extrinsic camera parameters.
g_M_c1 = np.matrix(
    [[1., 0., 0., 0.035],
     [0., 1., 0., 0.072],
     [0., 0., 1., 0.075],
     [0., 0., 0., 1.]])
g_M_c1 = np.matrix( #FIXME: disabled as long as it has not been checked.
    [[1., 0., 0., 0.],
     [0., 1., 0., 0.],
     [0., 0., 1., 0.],
     [0., 0., 0., 1.]])

# Frames re-orientation:
# Z = depth (increase from near to far)
# X = increase from left to right
# Y = increase from top to bottom
c1_M_c = np.matrix(
    [[ 0.,  0.,  1., 0.],
     [-1.,  0.,  0., 0.],
     [ 0., -1.,  0., 0.],
     [ 0.,  0.,  0., 1.]])


g_M_c = matrixToTuple(g_M_c1 * c1_M_c)

plug(robot.dynamic.gaze, w_M_c.positionIN)
plug(robot.dynamic.Jgaze, w_M_c.jacobianIN)
w_M_c.setTransformation(g_M_c)


# Localization
l = Localizer('localizer')

sensors = [w_M_c]

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


def split(P):
    return (P[0], P[1], P[2])

def P(sensorPosition, referencePoint):
    #FIXME: replace referencePoint by landmark
    referencePoint_ = np.inner(inverseHomogeneousMatrix(sensorPosition),
                               referencePoint)
    (X, Y, Z) = split(referencePoint_)

    x = u0 + f * px * X / Z
    y = v0 + f * py * Y / Z
    return np.array([x, y], dtype=np.float)

def dP(sensorPosition, referencePoint):
    proj = P(sensorPosition, referencePoint)

    referencePoint_ = np.inner(inverseHomogeneousMatrix(sensorPosition),
                               referencePoint)
    (x, y) = (proj[0], proj[1])
    (X, Y, Z) = split(referencePoint_)

    Lx = np.matrix(
        [[-1. / Z,  0.,     x / Z, x * y,     -(1 + (x * x)), y],
         [0.,      -1. / Z, y / Z, 1 + (y * y), -x * y,      -x]],
        dtype=np.float)
    return Lx


###############################

def XYThetaToHomogeneousMatrix(error):
    (offsetX, offsetY, offsetTheta) = (error[0], error[1], error[2])
    return np.matrix([
            [cos (offsetTheta), -sin(offsetTheta), 0., offsetX],
            [sin (offsetTheta),  cos(offsetTheta), 0., offsetY],
            [               0.,                0., 1.,      0.],
            [               0.,                0., 0.,      1.]],
                     dtype = np.float)

# planned 3d position of landmarks in the world frame
landmarks = [
    # (0, 0)
    np.array([0.025+10. , 0.        , 1.2967   , 1.], dtype=np.float),
    # (-2, 0)
    np.array([0.025+10. , 0.+2.     , 1.2967   , 1.], dtype=np.float),
    # (0, -3)
    np.array([0.025+10. , 0.        , 1.2967+3., 1.], dtype=np.float),
    # (-4, -5)
    np.array([0.025+10. , 0.+4.     , 1.2967+5., 1.], dtype=np.float),
    ]


# Localizer setup.

plannedRobot = [0., 0., 0., 1.] # robot planned position
error =  np.array([0., 10., 0.]) # x, y, theta
Tmax = 2

realRobot = np.inner(XYThetaToHomogeneousMatrix(error), plannedRobot)

for i in xrange(len(landmarks)):
    l.add_landmark_observation('obs'+str(i))

for t in xrange(Tmax):
    T = t + 10
    print("--- T =", T, "---")
    realRobotCfg = list(robot.dynamic.position.value)
    realRobotCfg[0] = realRobot[0]; realRobotCfg[1] = realRobot[1]; realRobotCfg[5] = realRobot[2]
    robot.dynamic.position.value = tuple(realRobotCfg)
    robot.dynamic.gaze.recompute(T)
    robot.dynamic.Jgaze.recompute(T)
    w_M_c.position.recompute(T)
    w_M_c.jacobian.recompute(T)

    print("Planned robot position:", plannedRobot[0:3])
    print("Real robot position:", realRobot[0:3])
    print("Error:", error[0:3])

    print("S(q)")
    print(S(0,0))
    print("dS(q)/dq (position + rotation vector)^2")
    print(dS(0,0))

    # observed 3d position of landmarks in the world frame
    observed_landmarks = []
    for l_ in landmarks:
        observed_landmarks.append(np.inner(np.linalg.inv(XYThetaToHomogeneousMatrix(error)), l_))

    # Select, X Y, yaw only!
    correctedDofs = (1., 1., 0., 0., 0., 1.) + 30 * (0.,)

    for i in xrange(len(observed_landmarks)):
        observed_landmark = observed_landmarks[i]
        landmark = landmarks[i]
        obsName = 'obs' + str(i)

        print("Landmark:", obsName)
        print("\t position (world frame): ", landmark[0:3])
        print("\t observed position (world frame): ", observed_landmark[0:3])
        print("\t position (camera frame): ", np.inner(
                inverseHomogeneousMatrix(S(0,0)), landmark)[0:3])
        print("\t observed position (camera frame): ", np.inner(
                inverseHomogeneousMatrix(S(0,0)), observed_landmark)[0:3])
        print("\t P(landmark): ", P(S(0,0), landmark))
        print("\t dP(landmark):")
        print(dP(S(0,0), landmark))
        print("\t P(observed_landmark): ", P(S(0,0), observed_landmark))

        print("\t dP*dS:")
        print(dP(S(0,0), landmark)*dS(0,0))

        print("\t dP*dS (fvp):")
        fvp = FeatureVisualPoint('fvp'+str(i))
        fvp.xy.value = (P(S(0,0), landmark)[0], P(S(0,0), landmark)[1])
        fvp.Z.value = np.inner(inverseHomogeneousMatrix(S(0,0)), landmark)[2]
        plug(w_M_c.jacobian, fvp.Jq)
        fvp.jacobian.recompute(T)
        print(np.matrix(fvp.jacobian.value)[0:2,0:6])

        l.signal(obsName + '_JfeatureReferencePosition').value = \
            matrixToTuple(dP(S(0, 0), landmark))
        l.signal(obsName + '_JsensorPosition').value = matrixToTuple(dS(0, 0))
        l.signal(obsName + '_weight').value = (1., 1.)

        l.signal(obsName + '_featureObservedPosition').value = \
            tuple(P(S(0,0), observed_landmark).tolist())
        l.signal(obsName + '_featureReferencePosition').value = \
            tuple(P(S(0,0), landmark).tolist())
        # Select (x, y, yaw) only!
        l.signal(obsName + '_correctedDofs').value = correctedDofs

        print("\n")

    l.configurationOffset.recompute(T)

    print("\n\nOffset (x, y, theta):")
    print(l.configurationOffset.value)
    print("\n")

    # planned position in the real position frame
    # pP = pMr rP
    offset = XYThetaToHomogeneousMatrix(l.configurationOffset.value)
    correction = np.linalg.inv(offset)

    rectifiedPosition = np.inner(correction, realRobot)
    print("Rectified position:", rectifiedPosition[0:3])
    print("...should be:", plannedRobot[0:3])
    print("...delta norm = ", np.linalg.norm(plannedRobot[0:3]-rectifiedPosition[0:3]))

    print("\n\n")

    for i in xrange(len(observed_landmarks)):
        observed_landmark = observed_landmarks[i]
        landmark = landmarks[i]
        obsName = 'obs' + str(i)

        rectified = np.inner(offset, observed_landmark)

        pRect = P(S(0,0), rectified)
        p = P(S(0,0), landmark)

        print("Landmark after rectification:", obsName)
        print("\t rectified position: ", rectified)
        print("\t P(rectified): ", pRect)
        print("\t P(planned pos): ", p)
        if np.linalg.norm(p - pRect) > 1e-3:
            print("\t WRONG: delta =", np.linalg.norm(p - pRect))


    # Update robot position and error!
    realRobot = rectifiedPosition
    error = map(lambda (e1, e2): e1 - e2, zip(plannedRobot, realRobot))

print("Final position:", realRobot[0:3])
