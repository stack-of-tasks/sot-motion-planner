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

from dynamic_graph import plug
from dynamic_graph.sot.dynamics.hrp2 import Hrp2Laas
from dynamic_graph.sot.motion_planner.feet_follower import SwayMotionCorrection

from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.math_small_entities import Derivator_of_Matrix
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags

from dynamic_graph.sot.dynamics.tools import robot as r, solver, timeStep, clt

smc = SwayMotionCorrection('smc')

robotDim = len(r.halfSitting)
robot = r.device

# --- DYN ----------------------------------------------------------------------
hrp2_10_pkgdatarootdir = "/home/thomas/profiles/laas/install/unstable/share/hrp2-10"
hrp2_14_pkgdatarootdir = "/home/thomas/profiles/laas/install/unstable/share/hrp2-14"

modelDir = hrp2_14_pkgdatarootdir
xmlDir = hrp2_14_pkgdatarootdir
modelName = 'HRP2JRLmainsmall.wrl'
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath = xmlDir + '/HRP2LinkJointRankSmall.xml'

dyn = r.dynamic
#dyn.inertiaRotor.value = inertiaRotor[robotName]
#dyn.gearRatio.value = gearRatio[robotName]

plug(robot.state,dyn.position)
dyn.velocity.value = robotDim*(0.,)
dyn.acceleration.value = robotDim*(0.,)

dyn.ffposition.unplug()
dyn.ffvelocity.unplug()
dyn.ffacceleration.unplug()
robot.control.unplug()

# --- PG ---------------------------------------------------------
from dynamic_graph.sot.pattern_generator import PatternGenerator,Selector

pg = PatternGenerator('pg')
pg.setVrmlDir(modelDir+'/')
pg.setVrml(modelName)
pg.setXmlSpec(specificitiesPath)
pg.setXmlRank(jointRankPath)
pg.buildModel()

# Standard initialization
pg.parseCmd(":samplingperiod 0.005")
pg.parseCmd(":previewcontroltime 1.6")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":omega 0.0")
pg.parseCmd(":stepheight 0.05")
pg.parseCmd(":singlesupporttime 0.780")
pg.parseCmd(":doublesupporttime 0.020")
pg.parseCmd(":armparameters 0.5")
pg.parseCmd(":LimitsFeasibility 0.0")
pg.parseCmd(":ZMPShiftParameters 0.015 0.015 0.015 0.015")
pg.parseCmd(":TimeDistributeParameters 2.0 3.5 1.0 3.0")
pg.parseCmd(":UpperBodyMotionParameters 0.0 -0.5 0.0")
pg.parseCmd(":comheight 0.814")
pg.parseCmd(":SetAlgoForZmpTrajectory Morisawa")

plug(dyn.position,pg.position)
plug(dyn.com,pg.com)
pg.motorcontrol.value = robotDim*(0,)
pg.zmppreviouscontroller.value = (0,0,0)

pg.initState()

# --- PG INIT FRAMES ---
geom = Dynamic("geom")
geom.setFiles(modelDir, modelName,specificitiesPath,jointRankPath)
geom.parse()
geom.createOpPoint('rf','right-ankle')
geom.createOpPoint('lf','left-ankle')
plug(dyn.position,geom.position)
geom.ffposition.value = 6*(0,)
geom.velocity.value = robotDim*(0,)
geom.acceleration.value = robotDim*(0,)

# --- Selector of Com Ref: when pg is stopped, pg.inprocess becomes 0
comRef = Selector('comRef'
                    ,['vector','ref',dyn.com,pg.comref])
plug(pg.inprocess,comRef.selec)

selecSupportFoot = Selector('selecSupportFoot'
                              ,['matrixHomo','pg_H_sf',pg.rightfootref,pg.leftfootref]
                              ,['matrixHomo','wa_H_sf',geom.rf,geom.lf]
                              )
plug(pg.SupportFoot,selecSupportFoot.selec)
sf_H_wa = Inverse_of_matrixHomo('sf_H_wa')
plug(selecSupportFoot.wa_H_sf,sf_H_wa.sin)
pg_H_wa = Multiply_of_matrixHomo('pg_H_wa')
plug(selecSupportFoot.pg_H_sf,pg_H_wa.sin1)
plug(sf_H_wa.sout,pg_H_wa.sin2)

# --- Compute the ZMP ref in the Waist reference frame.
wa_H_pg = Inverse_of_matrixHomo('wa_H_pg')
plug(pg_H_wa.sout,wa_H_pg.sin)
wa_zmp = Multiply_matrixHomo_vector('wa_zmp')
plug(wa_H_pg.sout,wa_zmp.sin1)
plug(pg.zmpref,wa_zmp.sin2)
# Connect the ZMPref to OpenHRP in the waist reference frame.
pg.parseCmd(':SetZMPFrame world')
plug(wa_zmp.sout,robot.zmp)

# ---- TASKS -------------------------------------------------------------------

# ---- WAIST TASK ---
taskWaist=MetaTask6d('waist',dyn,'waist','waist')

# Build the reference waist pos homo-matrix from PG.
waistReferenceVector = Stack_of_vector('waistReferenceVector')
plug(pg.initwaistposref,waistReferenceVector.sin1)
plug(pg.initwaistattref,waistReferenceVector.sin2)
waistReferenceVector.selec1(0,3)
waistReferenceVector.selec2(0,3)
waistReference=PoseRollPitchYawToMatrixHomo('waistReference')
plug(waistReferenceVector.sout,waistReference.sin)
plug(waistReference.sout,taskWaist.featureDes.position)

taskWaist.feature.selec.value = '011100'
taskWaist.task.controlGain.value = 5

# --- TASK COM ---
featureCom = FeatureGeneric('featureCom')
plug(dyn.com,featureCom.errorIN)
plug(dyn.Jcom,featureCom.jacobianIN)
featureComDes = FeatureGeneric('featureComDes')
featureCom.sdes.value = 'featureComDes'
plug(comRef.ref,featureComDes.errorIN)
featureCom.selec.value = '011'

taskComPD = TaskPD('taskComPD')
taskComPD.add('featureCom')
plug(pg.dcomref,featureComDes.errordotIN)
plug(featureCom.errordot,taskComPD.errorDot)
taskComPD.controlGain.value = 40
taskComPD.setBeta(-1)

# --- TASK RIGHT FOOT
# Task right hand
taskRF=MetaTask6d('rf',dyn,'rf','right-ankle')
taskLF=MetaTask6d('lf',dyn,'lf','left-ankle')

plug(pg.rightfootref,taskRF.featureDes.position)
taskRF.task.controlGain.value = 5
plug(pg.leftfootref,taskLF.featureDes.position)
taskLF.task.controlGain.value = 5

# ---- SOT ---------------------------------------------------------------------
# The solver SOTH of dyninv is used, but normally, the SOT solver should be sufficient
sot = solver.sot
sot.push(taskWaist.task.name)
sot.push(taskRF.task.name)
sot.push(taskLF.task.name)
sot.push(taskComPD.name)

plug(sot.control,robot.control)

# --- HERDT PG AND START -------------------------------------------------------
# Set the algorithm generating the ZMP reference trajectory to Herdt's one.
pg.parseCmd(':SetAlgoForZmpTrajectory Herdt')
pg.parseCmd(':doublesupporttime 0.1')
pg.parseCmd(':singlesupporttime 0.7')
# When velocity reference is at zero, the robot stops all motion after n steps
pg.parseCmd(':numberstepsbeforestop 4')
# Set constraints on XY
pg.parseCmd(':setfeetconstraint XY 0.09 0.06')

# The next command must be runned after a OpenHRP.inc ... ???
# Start the robot with a speed of 0.1 m/0.8 s.
pg.parseCmd(':HerdtOnline 0.1 0.0 0.0')

# You can now modifiy the speed of the robot using set pg.velocitydes [3]( x, y, yaw)
pg.velocitydes.value =(0.1,0.0,0.0)

# --- TRACER -----------------------------------------------------------------
from dynamic_graph.tracer import *
from dynamic_graph.tracer_real_time import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

#tr.add(dyn.name+'.ffposition','ff')
tr.add(taskRF.featureDes.name+'.position','refr')
tr.add(taskLF.featureDes.name+'.position','refl')


# smc
plug(r.dynamic.com, smc.wMcom)
plug(r.dynamic.waist, smc.wMwaist)
plug(r.frames['cameraBottomLeft'].position, smc.wMcamera)
plug(r.dynamic.Jcom, smc.Jcom)
plug(r.dynamic.Jwaist, smc.Jwaist)
plug(r.device.state, smc.qdot)

# retrieve cmo
from dynamic_graph.ros import Ros
ros = Ros(r)
ros.rosExport.add('matrixHomoStamped', "cMo", "/object_position")
plug(ros.rosExport.cMo, smc.cMo)
plug(ros.rosExport.cMoTimestamp, smc.cMoTimestamp)

I = ((1., 0., 0., 0.),
     (0., 1., 0., 0.),
     (0., 0., 1., 0.),
     (0., 0., 0., 1.))

smc.cMo.value = I
smc.cMoTimestamp.value = (0., 0.)

cdMo = ((1., 0., 0., 1.),
        (0., 1., 0., 0.),
        (0., 0., 1., 0.),
        (0., 0., 0., 1.))

t = 0
smc.initialize(cdMo, t)


r.device.increment(timeStep)
r.device.increment(timeStep)
r.device.increment(timeStep)

# plug vdes
plug(pg.dcomref, smc.inputPgVelocity)
plug(smc.outputPgVelocity, pg.velocitydes)

###########################################
# TO BE REMOVED BEFORE RUNNING ON OPENHRP #
###########################################
# play
def play(maxIter = 4000, clt = None):
    print("started")

    t = timeStep * 3.
    # Main.
    #  Main loop
    for i in xrange(maxIter):
        r.device.increment(timeStep)
        t += timeStep

        smc.dbgE.recompute(t)
        smc.dbgCorrectedE.recompute(t)
        smc.dbgVelocityWithoutCorrection.recompute(t)
        smc.dbgcMoWithCorrection.recompute(t)

        print("--- {0} ---".format(t))
        print("input velocity: {0}".format(smc.inputPgVelocity.value))
        print("output velocity: {0}".format(smc.outputPgVelocity.value))
        print("E: {0}".format(smc.dbgE.value))
        print("corrected E: {0}".format(smc.dbgCorrectedE.value))
        print("vel w/o correction: {0}".format(smc.dbgVelocityWithoutCorrection.value))
        print("cMo w correction: {0}".format(smc.dbgcMoWithCorrection.value))

        if clt:
            clt.updateElementConfig(
                'hrp', r.smallToFull(r.device.state.value))

play(clt = clt)
