#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright 2011, Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST
# Copyright 2013, Olivier Stasse, LAAS, CNRS
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

print("Run legs_follower_graph.py  v2.0.")
import time

from dynamic_graph import plug

print "import from sot.core"
from dynamic_graph.sot.core import FeatureGeneric, Task, FeaturePosture, MatrixConstant, RobotSimu
from dynamic_graph.sot.core.joint_trajectory_entity import SotJointTrajectoryEntity

print "import from sot.motion_planner"
from dynamic_graph.sot.motion_planner import LegsError, WaistError

print "import TracerRealTime"
from dynamic_graph.tracer_real_time import TracerRealTime

halfSitting = (0.0, 0.0, 0.648702, 0.0, 0.0, 0.0,
 0.0, 0.0, -0.45378600000000002, 0.87266500000000002, -0.418879, 0.0,
 0.0, 0.0, -0.45378600000000002, 0.87266500000000002, -0.418879, 0.0,
 0.0, 0.0,
 0.0, 0.0,
 0.60039, -0.17453, 0.0, -1.75057, 0.0, 0.0, 0.09948,
 0.60039, 0.17453, 0.0, -1.75057, 0.0, 0.0, 0.09948)
#0.261799, -0.17452999999999999, 0.0, -0.52359900000000004, 0.0, 0.0, 0.10000000000000001,
#0.261799, 0.17452999999999999, 0.0, -0.52359900000000004, 0.0, 0.0, 0.10000000000000001)


def oneVector(i,statelength):
    r = [0.,] * statelength
    r[i] = 1.
    return tuple(r)

def zeroVector(statelength):
    r = [0.,] * statelength
    return tuple(r)

class LegsFollowerGraph(object):

    legsFollower = None

    postureTask = None
    postureFeature = None
    postureFeatureDes = None
    postureError = None

    legsTask = None
    legsFeature = None
    legsFeatureDes = None
    legsError = None

    waistTask = None
    waistFeature = None
    waistFeatureDes = None
    waistError = None

    trace = None

    def __init__(self, robot, solver, ros, trace = None, postureTaskDofs = None):
        print("Constructor of LegsFollower Graph")
        self.robot = robot
        self.solver = solver
        self.ros = ros
	self.legsFollower = SotJointTrajectoryEntity('legs-follower')
        initTrajstr="((0,(1230.0,690059052.0),Trajectory_0),"
        initTrajstr=initTrajstr+"(RLEG_JOINT0,RLEG_JOINT1,RLEG_JOINT2,RLEG_JOINT3,RLEG_JOINT4,RLEG_JOINT5,"
        initTrajstr=initTrajstr+"LLEG_JOINT0,LLEG_JOINT1,LLEG_JOINT2,LLEG_JOINT3,LLEG_JOINT4,LLEG_JOINT5,"
        initTrajstr=initTrajstr+"COM_X,COM_Y,COM_THETA,COP_X,COP_Y),"
        initTrajstr=initTrajstr+"(((0.0,0.0,-0.453786,0.872665,-0.418879,0.0,0.0,0.0,-0.453786,0.872665,-0.418879,0.0,0.0,0.0,0.0,0.0,0.0)"
        initTrajstr=initTrajstr+",(),(),())))"
        self.legsFollower.initTraj(initTrajstr) 
        self.statelength = len(robot.device.state.value)

        # Plug ros in the legs follower entity
        # We assume that a poseTrajectory has been initialized previously
        plug(self.ros.rosExport.poseTrajectory,self.legsFollower.trajectoryIN)
        plug(self.legsFollower.seqid,self.ros.rosImport.seqId)  

 	# Initialize the posture task.
	print("Posture Task")
        self.postureTaskDofs = postureTaskDofs
        if not self.postureTaskDofs:
            self.postureTaskDofs = []
            for i in xrange(len(robot.halfSitting) - 6):
                # Disable legs dofs.
                if i < 12: #FIXME: not generic enough
                    self.postureTaskDofs.append((i + 6, False))
                else:
                    self.postureTaskDofs.append((i + 6, True))
        
        # This part is taken from feet_follower_graph
        self.postureTask = Task(self.robot.name + '_posture')
        
        self.postureFeature = FeaturePosture(self.robot.name + '_postureFeature')
        plug(self.robot.device.state, self.postureFeature.state)

        posture = list(self.robot.halfSitting)
        self.postureFeature.posture.value = tuple(posture)
        for (dof, isEnabled) in self.postureTaskDofs:
            self.postureFeature.selectDof(dof, isEnabled)
        self.postureTask.add(self.postureFeature.name)
        self.postureTask.controlGain.value = 2.

	# Initialize the waist follower task.
	print("Waist Task")
        self.robot.mTasks['waist'].feature.selec.value = '111111'
        plug(self.legsFollower.waist, self.robot.mTasks['waist'].featureDes.position)
        self.robot.mTasks['waist'].task.controlGain.value = 1.

	# Initialize the legs follower task.
	print("Legs Task")
        self.legsTask = Task(self.robot.name + '_legs')
        self.legsFeature = FeatureGeneric(self.robot.name + '_legsFeature')
        legsFeatureDesName = self.robot.name + '_legsFeatureDes'
        self.legsFeatureDes = FeatureGeneric(legsFeatureDesName)
        self.legsError = LegsError('LegsError')
        plug(self.robot.device.state, self.legsError.state)

        plug(self.legsFollower.position,self.legsFeatureDes.errorIN)
        self.legsFeature.jacobianIN.value = self.legsJacobian()

        self.legsFeature.setReference(legsFeatureDesName)
        plug(self.legsError.error, self.legsFeature.errorIN)            

        self.legsTask.add(self.legsFeature.name)
        self.legsTask.controlGain.value = 5.

	#CoM task
        print("Com Task")
        print (0., 0., self.robot.dynamic.com.value[2])
	self.robot.mTasks['com'].task.controlGain.value = 50.
        self.robot.mTasks['com'].featureDes.errorIN.value =  (0., 0., self.robot.dynamic.com.value[2])
        self.robot.mTasks['com'].feature.selec.value = '110'
	plug(self.legsFollower.com, self.robot.mTasks['com'].featureDes.errorIN)

        # Plug the legs follower zmp output signals.
        plug(self.legsFollower.zmp, self.robot.device.zmp)

        # Prepare stack
	self.initStack()
        
    def initStack(self):
        self.solver.sot.clear() 
        print("robot.contactLF: " + str(dir(self.robot.contactLF)))
        #self.solver.push(self.robot.taskLim)
        self.solver.sot.push(self.robot.mTasks['waist'].task.name)
        self.solver.sot.push(self.legsTask.name)
        self.solver.sot.push(self.postureTask.name)
        self.solver.sot.push(self.robot.mTasks['com'].task.name)
        
	print self.solver.sot.dispStack()

    def legsJacobian(self):
        j = []
        for i in xrange(12):
            j.append(oneVector(6+i,self.statelength)) 
        return tuple(j)

    def waistJacobian(self):
        j = []
        for i in xrange(6):
            j.append(oneVector(i,self.statelength)) 
        return tuple(j)

    def postureJacobian(self):
        j = []
        for i in xrange(self.statelength):
            if i >= 6 + 2 * 6:
                j.append(oneVector(i))
            if i == 3 or i == 4:
                j.append(zeroVector())
        return tuple(j)

    def computeDesiredValue(self):
        e = self.robot.halfSitting
        #e = halfSitting
        e_ = [e[3], e[4]]
        offset = 6 + 2 * 6
        for i in xrange(len(e) - offset):
            e_.append(e[offset + i])
        return tuple(e_)
    
    def canStart(self):
        securityThreshold = 1e-3
        return (self.postureTask.error.value <=
                (securityThreshold,) * len(self.postureTask.error.value))

    def setupTrace(self):
	self.trace = TracerRealTime('trace')
	self.trace.setBufferSize(2**20)
	self.trace.open('/tmp/','legs_follower_','.dat')
	
	self.trace.add('legs-follower.com', 'com')
	self.trace.add('legs-follower.zmp', 'zmp')
	self.trace.add('legs-follower.waist', 'waist')
	self.trace.add('legs-follower.position', 'position')
	self.trace.add(self.robot.device.name + '.state', 'state')
	self.trace.add(self.legsTask.name + '.error', 'errorLegs')
        self.trace.add(self.robot.mTasks['com'].task.name + '.error', 'errorCom')

        self.trace.add(self.robot.dynamic.name + '.left-ankle',
                       self.robot.dynamic.name + '-left-ankle')
        self.trace.add(self.robot.dynamic.name + '.right-ankle',
                       self.robot.dynamic.name + '-right-ankle')


	# Recompute trace.triger at each iteration to enable tracing.
	self.robot.device.after.addSignal('legs-follower.zmp')
	self.robot.device.after.addSignal('legs-follower.seqid')
	#self.robot.device.after.addSignal('legs-follower.outputYaw')
        self.robot.device.after.addSignal(self.robot.dynamic.name + '.left-ankle')
	self.robot.device.after.addSignal(self.robot.dynamic.name + '.right-ankle')
	self.robot.device.after.addSignal('trace.triger')
	return

    def plugPlanner(self):
        print("Plug planner.")
	return

    def plugPlannerWithoutMocap(self):
        print("Plug planner without mocap.")
	return

    def plugViewer(self):
        print("Plug viewer.")
	return

    def plug(self):
	self.plugPlanner()
	self.plugViewer()
	return

    def readMocap(self):
	return;

    def start(self):
        if not self.canStart():
            print("Robot has not yet converged to the initial position,"
                  " please wait and try again.")
            return

        print("Start.")
	self.postureTask.controlGain.value = 180.
        #self.waistTask.controlGain.value = 90.
	self.legsTask.controlGain.value = 180.
	self.robot.mTasks['com'].task.controlGain.value = 180.
	self.robot.mTasks['waist'].task.controlGain.value = 45.

	self.setupTrace()
	self.trace.start()
	return

    def stop(self):
	self.trace.dump()
	return
