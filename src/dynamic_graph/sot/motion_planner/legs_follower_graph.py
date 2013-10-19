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

print("Run legs_follower_graph.py  v2.0.")

import time

print "import from dynamics.tools"
from dynamic_graph.sot.dynamics.tools import *

print "import from sot.core"
from dynamic_graph.sot.core import \
    FeatureGeneric, MatrixConstant, RobotSimu

print "import from sot.motion_planner"
from dynamic_graph.sot.motion_planner import LegsFollower, PostureError, LegsError, WaistError

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


def oneVector(i):
    r = [0.,] * 36
    r[i] = 1.
    return tuple(r)

def zeroVector():
    r = [0.,] * 36
    return tuple(r)

class LegsFollowerGraph:

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

    def __init__(self, robot, solver, trace = None):

	self.legsFollower = LegsFollower('legs-follower')

 	# Initialize the posture task.
	print("Posture Task")
        self.postureTask = Task(robot.name + '_posture')
        self.postureFeature = FeatureGeneric(robot.name + '_postureFeature')
        self.postureFeatureDes = FeatureGeneric(robot.name + '_postureFeatureDes')
        self.postureError = PostureError('PostureError')
        plug(robot.device.state, self.postureError.state)
        plug(self.postureError.error, self.postureFeature.errorIN)
        self.postureFeature.jacobianIN.value = self.postureJacobian()
        self.postureFeatureDes.errorIN.value = self.computeDesiredValue()
        self.postureFeature.sdes.value = self.postureFeatureDes
        self.postureTask.add(self.postureFeature.name)
        self.postureTask.controlGain.value = 2.

	# Initialize the waist follower task.
	print("Waist Task")
        robot.features['waist'].selec.value = '111111'
        plug(self.legsFollower.waist, robot.features['waist'].reference)
        robot.tasks['waist'].controlGain.value = 1.

        #self.waistTask = Task(robot.name + '_waist')
        #self.waistFeature = FeatureGeneric(robot.name + '_waistFeature')
        #self.waistFeatureDes = FeatureGeneric(robot.name + '_waistsFeatureDes')
        #self.waistError = WaistError('WaistError')
        #plug(robot.device.state, self.waistError.state)
        #plug(self.waistError.error, self.waistFeature.errorIN)
        #self.waistFeature.jacobianIN.value = self.waistJacobian()
	#plug(self.legsFollower.waist, self.waistFeatureDes.errorIN)
        #self.waistFeature.sdes.value = self.waistFeatureDes
        #self.waistTask.add(self.waistFeature.name)
        #self.waistTask.controlGain.value = 1.

	# Initialize the legs follower task.
	print("Legs Task")
        self.legsTask = Task(robot.name + '_legs')
        self.legsFeature = FeatureGeneric(robot.name + '_legsFeature')
        self.legsFeatureDes = FeatureGeneric(robot.name + '_legsFeatureDes')
        self.legsError = LegsError('LegsError')
        plug(robot.device.state, self.legsError.state)
        plug(self.legsError.error, self.legsFeature.errorIN)
        self.legsFeature.jacobianIN.value = self.legsJacobian()
	plug(self.legsFollower.ldof, self.legsFeatureDes.errorIN)
        self.legsFeature.sdes.value = self.legsFeatureDes
        self.legsTask.add(self.legsFeature.name)
        self.legsTask.controlGain.value = 5.


	#CoM task
        print("Com Task")
        print (0., 0., robot.dynamic.com.value[2])
	robot.comTask.controlGain.value = 50.
        robot.featureComDes.errorIN.value = (0., 0., robot.dynamic.com.value[2])
        robot.featureCom.selec.value = '111'
	plug(self.legsFollower.com, robot.featureComDes.errorIN)

        # Plug the legs follower zmp output signals.
        plug(self.legsFollower.zmp, robot.device.zmp)


	solver.sot.remove(robot.comTask.name)

	print("Push in solver.")
        solver.sot.push(self.legsTask.name)
        solver.sot.push(self.postureTask.name)
	solver.sot.push(robot.tasks['waist'].name)
        solver.sot.push(robot.comTask.name)
        
        solver.sot.remove(robot.tasks['left-ankle'].name)
	solver.sot.remove(robot.tasks['right-ankle'].name)


	print solver.sot.display()

        print("Tasks added in solver.\n")
	print("Command are : \n - f.plug()\n - f.plugViewer()\n - f.plugPlanner()\n"
              " - f.plugPlannerWithoutMocap()\n - f.start()\n - f.stop()\n - f.readMocap()\n")


    def legsJacobian(self):
        j = []
        for i in xrange(12):
            j.append(oneVector(6+i)) 
        return tuple(j)

    def waistJacobian(self):
        j = []
        for i in xrange(6):
            j.append(oneVector(i)) 
        return tuple(j)

    def postureJacobian(self):
        j = []
        for i in xrange(36):
            if i >= 6 + 2 * 6:
                j.append(oneVector(i))
            if i == 3 or i == 4:
                j.append(zeroVector())
        return tuple(j)

    def computeDesiredValue(self):
        e = robot.halfSitting
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
	self.trace.setBufferSize(2**26)
	self.trace.open('/tmp/','trace_','.dat')
	
	self.trace.add('legs-follower.com', 'com')
	self.trace.add('legs-follower.zmp', 'zmp')
	self.trace.add('legs-follower.ldof', 'ldof')
	self.trace.add('legs-follower.waist', 'waist')
	self.trace.add(robot.device.name + '.state', 'state')
	self.trace.add(self.legsTask.name + '.error', 'errorLegs')
        self.trace.add(robot.comTask.name + '.error', 'errorCom')

        self.trace.add('legs-follower.outputStart','start')
        self.trace.add('legs-follower.outputYaw','yaw')
        self.trace.add('corba.planner_steps','steps')
        self.trace.add('corba.planner_outputGoal','goal')
        self.trace.add('corba.waist','waistMocap')
	self.trace.add('corba.left-foot','footMocap')
        self.trace.add('corba.table','tableMocap')
        self.trace.add('corba.bar','barMocap')
        self.trace.add('corba.chair','chairMocap')
	self.trace.add('corba.helmet','helmetMocap')
	self.trace.add('corba.planner_outputObs','obstacles')

        self.trace.add(robot.dynamic.name + '.left-ankle',
                       robot.dynamic.name + '-left-ankle')
        self.trace.add(robot.dynamic.name + '.right-ankle',
                       robot.dynamic.name + '-right-ankle')


	# Recompute trace.triger at each iteration to enable tracing.
	robot.device.after.addSignal('legs-follower.zmp')
	robot.device.after.addSignal('legs-follower.outputStart')
	robot.device.after.addSignal('legs-follower.outputYaw')
	robot.device.after.addSignal('corba.planner_steps')
	robot.device.after.addSignal('corba.planner_outputGoal')
	robot.device.after.addSignal('corba.waist')
	robot.device.after.addSignal('corba.left-foot')
	robot.device.after.addSignal('corba.table')
	robot.device.after.addSignal('corba.bar')
	robot.device.after.addSignal('corba.chair')
	robot.device.after.addSignal('corba.helmet')
	robot.device.after.addSignal('corba.planner_outputObs')
        robot.device.after.addSignal(robot.dynamic.name + '.left-ankle')
	robot.device.after.addSignal(robot.dynamic.name + '.right-ankle')
	robot.device.after.addSignal('trace.triger')
	return

    def plugPlanner(self):
        print("Plug planner.")
	plug(corba.planner_radQ, self.legsFollower.inputRef)
	plug(self.legsFollower.outputStart, corba.planner_inputStart)
	plug(corba.waistTimestamp, corba.planner_timestamp)
	plug(corba.table, corba.planner_table)
	plug(corba.chair, corba.planner_chair)
	plug(corba.bar, corba.planner_bar)
	plug(corba.waist, corba.planner_waist)
	plug(corba.helmet, corba.planner_inputGoal)
        plug(corba.torus1, corba.planner_torus1)
	plug(corba.torus2, corba.planner_torus2)
        plug(corba.signal('left-foot'), corba.planner_foot)
        plug(corba.signal('left-footTimestamp'), corba.planner_footTimestamp)
	return

    def plugPlannerWithoutMocap(self):
        print("Plug planner without mocap.")
	plug(corba.planner_radQ, self.legsFollower.inputRef)
	plug(self.legsFollower.outputStart, corba.planner_inputStart)
	return

    def plugViewer(self):
        print("Plug viewer.")
	plug(self.legsFollower.ldof, corba.viewer_Ldof)
	plug(self.legsFollower.outputStart, corba.viewer_Start)
	plug(self.legsFollower.com, corba.viewer_Com)
	plug(self.legsFollower.outputYaw, corba.viewer_Yaw)
	plug(corba.planner_steps, corba.viewer_Steps)
	plug(corba.planner_outputGoal, corba.viewer_Goal)
	plug(corba.table, corba.viewer_Table)
	plug(corba.chair, corba.viewer_Chair)
	plug(corba.bar, corba.viewer_Bar)
        plug(corba.torus1, corba.viewer_Torus1)
	plug(corba.torus2, corba.viewer_Torus2)
	plug(corba.waist, corba.viewer_Waist)
	plug(corba.planner_outputLabyrinth, corba.viewer_Labyrinth)
	plug(corba.planner_outputObs, corba.viewer_Obs)
	return

    def plug(self):
	self.plugPlanner()
	self.plugViewer()
	return

    def readMocap(self):
	print "Table : " 
	print corba.table.value
	print "Waist : " 
	if len(corba.waist.value)<3:
	    corba.waist.value = (0,0,0)
	print corba.waist.value
	print "Helmet : " 
	print corba.helmet.value
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
	robot.comTask.controlGain.value = 180.
	robot.tasks['waist'].controlGain.value = 45.

	self.setupTrace()
	self.trace.start()
        self.legsFollower.start()
	return

    def stop(self):
	self.legsFollower.stop()
	self.trace.dump()
	return


f = LegsFollowerGraph()
while True:
    try:
        corba.planner_radQ
        break
    except:
        time.sleep(0.01)

f.plug()
