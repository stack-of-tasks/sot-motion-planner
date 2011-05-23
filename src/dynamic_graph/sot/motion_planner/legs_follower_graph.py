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

print("Run legs_follower_graph.py  v1.7.")

print "import from dynamics.tools"
from dynamic_graph.sot.dynamics.tools import *

print "import robot, solver"
from __main__ import robot, solver

print "import from sot.core"
from dynamic_graph.sot.core import \
    FeatureGeneric, Task, MatrixConstant, RobotSimu

print "import from sot.motion_planner"
from dynamic_graph.sot.motion_planner import LegsFollower, PostureError, LegsError, WaistError

print "import TracerRealTime"
from dynamic_graph.tracer_real_time import TracerRealTime

print "import CorbaServer"
from dynamic_graph.corba_server import CorbaServer
corba = CorbaServer('corba')
#corba.displaySignals()


def oneVector(i):
    r = [0.,] * 36
    r[i] = 1.
    return tuple(r)

def zeroVector():
    r = [0.,] * 36
    return tuple(r)

class LFollower:

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

    def __init__(self):

	self.legsFollower = LegsFollower('legs-follower')

	self.trace = TracerRealTime('trace')
	self.trace.setBufferSize(2**20)
	self.trace.open('/tmp/','trace_','.dat')

	for s in robot.OperationalPoints + ["com", "zmp"]:
            self.trace.add(robot.dynamic.name + '.' + s,
                           robot.dynamic.name + '-' + s)
            robot.device.after.addSignal(robot.dynamic.name + '.' + s)

        for s in robot.OperationalPoints:
            self.trace.add(
                robot.features[s]._reference.name + '.position',
                robot.features[s]._reference.name + '-position')
            robot.device.after.addSignal(
                robot.features[s]._reference.name + '.position')

            self.trace.add(
                robot.tasks[s].name + '.error',
                robot.tasks[s].name + '-error')
            robot.device.after.addSignal(
                robot.tasks[s].name + '.error')


        self.trace.add(robot.featureComDes.name + '.errorIN',
                       robot.featureComDes.name + '-errorIN')
        robot.device.after.addSignal(robot.featureComDes.name + '.errorIN')

        self.trace.add(robot.comTask.name + '.error',
                       robot.comTask.name + '-error')
        robot.device.after.addSignal(robot.comTask.name + '.error')

        self.trace.add(robot.device.name + '.zmp', robot.device.name + '-zmp')
        robot.device.after.addSignal(robot.device.name + '.zmp')



	# Recompute trace.triger at each iteration to enable tracing.
	robot.device.after.addSignal('trace.triger')

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
        self.postureTask.controlGain.value = 1.

	# Initialize the waist follower task.
	#print("Waist Task")
        #robot.features['waist'].selec.value = '111111'
        #plug(self.legsFollower.waist, robot.features['waist'].reference)
        #robot.tasks['waist'].controlGain.value = 1.

        self.waistTask = Task(robot.name + '_waist')
        self.waistFeature = FeatureGeneric(robot.name + '_waistFeature')
        self.waistFeatureDes = FeatureGeneric(robot.name + '_waistsFeatureDes')

        self.waistError = WaistError('WaistError')
        plug(robot.device.state, self.waistError.state)
        plug(self.waistError.error, self.waistFeature.errorIN)

        self.waistFeature.jacobianIN.value = self.waistJacobian()
	plug(self.legsFollower.waist, self.waistFeatureDes.errorIN)

        self.waistFeature.sdes.value = self.waistFeatureDes

        self.waistTask.add(self.waistFeature.name)
        self.waistTask.controlGain.value = 1.

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
        self.legsTask.controlGain.value = 1.


	#CoM task
	robot.comTask.controlGain.value = 5.
        robot.featureComDes.errorIN.value = (0., 0., robot.dynamic.com.value[2])
        robot.featureCom.selec.value = '111'
	plug(self.legsFollower.com, robot.featureComDes.errorIN)

        # Plug the legs follower zmp output signals.
        plug(self.legsFollower.zmp, robot.device.zmp)


	solver.sot.remove(robot.comTask.name)

	print("Push in solver.")
        solver.sot.push(self.legsTask.name)
        solver.sot.push(self.postureTask.name)
	solver.sot.push(robot.comTask.name)
        solver.sot.push(robot.tasks['waist'].name)

	solver.sot.remove(robot.tasks['left-ankle'].name)
	solver.sot.remove(robot.tasks['right-ankle'].name)


	print solver.sot.display()

        print("Tasks added in solver.\n")
	print("Command are : \n - f.plug()\n - f.plugViewer()\n - f.plugPlanner()\n - f.plugPlannerWithoutMocap()\n - f.start()\n - f.stop()\n - f.readMocap()\n")

    def legsJacobian(self):
	#print "legsJacobian"
        j = []
        for i in xrange(12):
	    #print 6+i
            j.append(oneVector(6+i)) 
        return tuple(j)

    def waistJacobian(self):
        j = []
        for i in xrange(6):
            j.append(oneVector(i)) 
        return tuple(j)

    def postureJacobian(self):
	#print "postureJacobian"
        j = []
        for i in xrange(36):
            #if i == 3 or i == 4 or i == 5 or i >= 6 + 2 * 6:
            if i >= 6 + 2 * 6:
                j.append(oneVector(i))
	    if i == 3 or i == 4 or i==5:
		j.append(zeroVector())
        return tuple(j)

    def computeDesiredValue(self):
        e = robot.halfSitting
        e_ = [e[3], e[4], e[5]]
        offset = 6 + 2 * 6
        for i in xrange(len(e) - offset):
		e_.append(e[offset + i])
        return tuple(e_)

    def canStart(self):
        securityThreshold = 1e-3
        return (self.postureTask.error.value <=
                (securityThreshold,) * len(self.postureTask.error.value))

    def setupTrace(self):
	self.trace.add('legs-follower.com', 'com')
	self.trace.add('legs-follower.zmp', 'zmp')
	self.trace.add('legs-follower.ldof', 'ldof')
	self.trace.add('legs-follower.waist', 'waist')
	self.trace.add(robot.device.name + '.state', 'state')
	self.trace.add(self.legsTask.name + '.error', 'errorLegs')
	self.trace.add(self.waistTask.name + '.error', 'errorWaist')
	self.trace.add(robot.comTask.name + '.error', 'errorCom')
	return

    def plugPlanner(self):
        print("Plug planner.")
	plug(corba.FR_radQ, self.legsFollower.inputRef)
	plug(self.legsFollower.outputStart, corba.FR_inputStart)
	plug(corba.tablePosition, corba.FR_table)
	plug(corba.waistPosition, corba.FR_waist)
	plug(corba.helmetPosition, corba.FR_inputGoal)
	return

    def plugPlannerWithoutMocap(self):
        print("Plug planner without mocap.")
	plug(corba.FR_radQ, self.legsFollower.inputRef)
	plug(self.legsFollower.outputStart, corba.FR_inputStart)
	return

    def plugViewer(self):
        print("Plug viewer.")
	plug(self.legsFollower.ldof, corba.viewer_inputLdof)
	plug(self.legsFollower.outputStart, corba.viewer_inputStart)
	plug(self.legsFollower.waist, corba.viewer_inputCom)
	plug(corba.FR_steps, corba.viewer_inputSteps)
	plug(corba.FR_outputGoal, corba.viewer_inputGoal)
	plug(corba.tablePosition, corba.viewer_inputTable)
	plug(corba.waistPosition, corba.viewer_inputWaist)
	return

    def plug(self):
	self.plugPlanner()
	self.plugViewer()
	return

    def readMocap(self):
	print "Table : " 
	print corba.tablePosition.value
	print "Waist : " 
	print corba.waistPosition.value
	print "Helmet : " 
	print corba.helmetPosition.value
	return;

    def start(self):
        if not self.canStart():
            print("Robot has not yet converged to the initial position,"
                  " please wait and try again.")
            return

        print("Start.")
	self.postureTask.controlGain.value = 10.
        self.waistTask.controlGain.value = 90.
	self.legsTask.controlGain.value = 180.
	robot.comTask.controlGain.value = 50.
#robot.tasks['waist'].controlGain.value = 1.

        self.waistTask = Task(robot.name + '_waist')
        self.waistFeature = FeatureGeneric(robot.name + '_waistFeature')
        self.waistFeatureDes = FeatureGeneric(robot.name + '_waistsFeatureDes')

        self.waistError = WaistError('WaistError')
        plug(robot.device.state, self.waistError.state)
        plug(self.waistError.error, self.waistFeature.errorIN)

        self.waistFeature.jacobianIN.value = self.waistJacobian()
	plug(self.legsFollower.waist, self.waistFeatureDes.errorIN)

        self.waistFeature.sdes.value = self.waistFeatureDes

        self.waistTask.add(self.waistFeature.name)
        self.waistTask.controlGain.value = 1.

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
        self.legsTask.controlGain.value = 1.


	#CoM task
	robot.comTask.controlGain.value = 5.
        robot.featureComDes.errorIN.value = (0., 0., robot.dynamic.com.value[2])
        robot.featureCom.selec.value = '111'
	plug(self.legsFollower.com, robot.featureComDes.errorIN)

        # Plug the legs follower zmp output signals.
        plug(self.legsFollower.zmp, robot.device.zmp)


	solver.sot.remove(robot.comTask.name)

	print("Push in solver.")
        solver.sot.push(self.legsTask.name)
        solver.sot.push(self.postureTask.name)
	solver.sot.push(robot.comTask.name)
        solver.sot.push(robot.tasks['waist'].name)

	solver.sot.remove(robot.tasks['left-ankle'].name)
	solver.sot.remove(robot.tasks['right-ankle'].name)


	print solver.sot.display()

        print("Tasks added in solver.\n")
	print("Command are : \n - f.plug()\n - f.plugViewer()\n - f.plugPlanner()\n - f.plugPlannerWithoutMocap()\n - f.start()\n - f.stop()\n - f.readMocap()\n")

    def legsJacobian(self):
	#print "legsJacobian"
        j = []
        for i in xrange(12):
	    #print 6+i
            j.append(oneVector(6+i)) 
        return tuple(j)

    def waistJacobian(self):
        j = []
        for i in xrange(6):
            j.append(oneVector(i)) 
        return tuple(j)

    def postureJacobian(self):
	#print "postureJacobian"
        j = []
        for i in xrange(36):
            #if i == 3 or i == 4 or i == 5 or i >= 6 + 2 * 6:
            if i >= 6 + 2 * 6:
                j.append(oneVector(i))
	    if i == 3 or i == 4 or i==5:
		j.append(zeroVector())
        return tuple(j)

    def computeDesiredValue(self):
        e = robot.halfSitting
        e_ = [e[3], e[4], e[5]]
        offset = 6 + 2 * 6
        for i in xrange(len(e) - offset):
		e_.append(e[offset + i])
        return tuple(e_)

    def canStart(self):
        securityThreshold = 1e-3
        return (self.postureTask.error.value <=
                (securityThreshold,) * len(self.postureTask.error.value))

    def setupTrace(self):
	self.trace.add('legs-follower.com', 'com')
	self.trace.add('legs-follower.zmp', 'zmp')
	self.trace.add('legs-follower.ldof', 'ldof')
	self.trace.add('legs-follower.waist', 'waist')
	self.trace.add(robot.device.name + '.state', 'state')
	self.trace.add(self.legsTask.name + '.error', 'errorLegs')
	self.trace.add(self.waistTask.name + '.error', 'errorWaist')
	self.trace.add(robot.comTask.name + '.error', 'errorCom')
	return

    def plugPlanner(self):
        print("Plug planner.")
	plug(corba.FR_radQ, self.legsFollower.inputRef)
	plug(self.legsFollower.outputStart, corba.FR_inputStart)
	plug(corba.tablePosition, corba.FR_table)
	plug(corba.waistPosition, corba.FR_waist)
	plug(corba.helmetPosition, corba.FR_inputGoal)
	return

    def plugPlannerWithoutMocap(self):
        print("Plug planner without mocap.")
	plug(corba.FR_radQ, self.legsFollower.inputRef)
	plug(self.legsFollower.outputStart, corba.FR_inputStart)
	return

    def plugViewer(self):
        print("Plug viewer.")
	plug(self.legsFollower.ldof, corba.viewer_inputLdof)
	plug(self.legsFollower.outputStart, corba.viewer_inputStart)
	plug(self.legsFollower.waist, corba.viewer_inputCom)
	plug(corba.FR_steps, corba.viewer_inputSteps)
	plug(corba.FR_outputGoal, corba.viewer_inputGoal)
	plug(corba.tablePosition, corba.viewer_inputTable)
	plug(corba.waistPosition, corba.viewer_inputWaist)
	return

    def plug(self):
	self.plugPlanner()
	self.plugViewer()
	return

    def readMocap(self):
	print "Table : " 
	print corba.tablePosition.value
	print "Waist : " 
	print corba.waistPosition.value
	print "Helmet : " 
	print corba.helmetPosition.value
	return;

    def start(self):
        if not self.canStart():
            print("Robot has not yet converged to the initial position,"
                  " please wait and try again.")
            return

        print("Start.")
	self.postureTask.controlGain.value = 10.
        self.waistTask.controlGain.value = 90.
	self.legsTask.controlGain.value = 180.
	robot.comTask.controlGain.value = 50.
        #robot.tasks['waist'].controlGain.value = 180.

	self.trace.start()
        self.legsFollower.start()
	return

    def stop(self):
	self.legsFollower.stop()
	self.trace.dump()
	return


f = LFollower()
