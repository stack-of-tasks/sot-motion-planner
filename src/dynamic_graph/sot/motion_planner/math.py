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
import numpy as np
from ..math import acos, atan2, cos, sin, pi, sqrt

# Random mathematics tools.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)


# Set roll and pitch angles to 0.
#
# Useful when making the assumption that the robot is walking on a flat
# ground to cancel sensor errors.
def CancelRollPitch(x):
    return XYThetaToHomogeneousMatrix(HomogeneousMatrixToXYTheta(x))

def XYThetaToHomogeneousMatrix(x):
    theta = x[2]
    return np.matrix(
        (( cos (theta),-sin (theta), 0., x[0]),
         ( sin (theta), cos (theta), 0., x[1]),
         (          0.,          0., 1., 0.),
         (          0.,          0., 0., 1.))
        )
def HomogeneousMatrixToXYZTheta(x):
    x = np.mat(x)
    return (x[0,3], x[1,3], x[2,3], atan2(x[1,0], x[0,0]))

def HomogeneousMatrixToXYTheta(x):
    x = np.mat(x)
    return (x[0,3], x[1,3], atan2(x[1,0], x[0,0]))

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
def yaw(rotationMatrix):
    return atan2(rotationMatrix[1,0], rotationMatrix[0,0])

def pitch(rotationMatrix):
    return atan2(-rotationMatrix[2,0],
                  sqrt(rotationMatrix[2,1]**2 + rotationMatrix[2,2]**2))

def roll(rotationMatrix):
    return atan2(rotationMatrix[2,1], rotationMatrix[2,2])

def tx(m):
    return m[0,3]
def ty(m):
    return m[1,3]
def tz(m):
    return m[2,3]

def pose(m):
    return [tx(m), ty(m), tz(m), roll(m), pitch(m), yaw(m)]

def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

def translationToSE3(t):
    return ((1., 0., 0., t[0]),
            (0., 1., 0., t[1]),
            (0., 0., 1., t[2]),
            (0., 0., 0., 1.  ))

def oneVector(i):
    r = [0.,] * 36
    r[i] = 1.
    return tuple(r)


def rollPitchYawToRotationMatrix(tx = 0., ty = 0., tz = 0.,
                                 roll = 0., pitch = 0., yaw = 0.):
    """Transformation order is roll then pitch then yaw then translation."""

    cr = cos(roll)
    cp = cos(pitch)
    cy = cos(yaw)

    sr = sin(roll)
    sp = sin(pitch)
    sy = sin(yaw)

    return np.matrix(
        [[cy * cp, cy * sp * sr - sy * sr, cy * sp * cr + sy * sp, tx],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, ty],
         [-sp    , cp * sr               , cp * cr               , tz],
         [0.     , 0.                    , 0.                    , 1.],],
        dtype = np.dtype(np.float)
        )

class Pose6d(object):
    x = 0.
    y = 0.
    z = 0.
    rx = 0.
    ry = 0.
    rz = 0.

    @staticmethod
    def fromRotationMatrix(m):
        x = m[0, 3]
        y = m[1, 3]
        z = m[2, 3]

        rx = roll(m)
        ry = pitch(m)
        rz = yaw(m)
        return Pose6d({'x': x, 'y': y, 'z': z, 'rx': rx, 'ry': ry, 'rz': rz})

    def __init__(self, yamlData):
        self.x = yamlData.get('x', 0.)
        self.y = yamlData.get('y', 0.)
        self.z = yamlData.get('z', 0.)
        self.rx = yamlData.get('rx', 0.)
        self.ry = yamlData.get('ry', 0.)
        self.rz = yamlData.get('rz', 0.)

    def rotationMatrix(self):
        return rollPitchYawToRotationMatrix(self.x , self.y , self.z,
                                            self.rx, self.ry, self.rz)

    def dgRotationMatrix(self):
        return matrixToTuple(self.rotationMatrix())

    def pose(self, rotation = True, translation = True):
        if not rotation or not translation:
            if rotation:
                return [0., 0., 0., self.rx, self.ry, self.rz]
            else:
                return [self.x, self.y, self.z, 0., 0., 0.]
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def __str__(self):
        return "Pose6d({0}, {1}, {2}, {3}, {4}, {5})".format(
            self.x, self.y, self.z, self.rx, self.ry, self.rz)
