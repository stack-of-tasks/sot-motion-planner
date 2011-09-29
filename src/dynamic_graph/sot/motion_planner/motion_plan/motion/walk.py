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
from dynamic_graph.sot.motion_planner.feet_follower_graph \
    import FeetFollowerAnalyticalPgGraph

from dynamic_graph.sot.motion_planner.math import *
from dynamic_graph.sot.motion_planner.motion_plan.tools import *

from dynamic_graph.sot.motion_planner.motion_plan.motion.abstract import *

class MotionWalk(Motion):
    yaml_tag = u'walk'

    footsteps = None
    waistFile = None
    gazeFile = None
    feetFollower = None

    def __init__(self, motion, yamlData, defaultDirectories):
        checkDict('interval', yamlData)
        checkDict('footsteps', yamlData)

        Motion.__init__(self, motion, yamlData)

        steps = convertToNPFootstepsStack(yamlData['footsteps'])
        self.footsteps = yamlData['footsteps']
        self.comZ = yamlData.get('comZ')

        #FIXME: handle multiple walk movement.
        motion.footsteps = yamlData['footsteps']

        self.waistFile = searchFile(yamlData.get('waist-trajectory'),
                                    defaultDirectories)
        self.gazeFile = searchFile(yamlData.get('gaze-trajectory'),
                                    defaultDirectories)
        self.zmpFile = searchFile(yamlData.get('zmp-trajectory'),
                                    defaultDirectories)
        self.feetFollower = FeetFollowerAnalyticalPgGraph(
            motion.robot, motion.solver, steps,
            waistFile = self.waistFile,
            gazeFile = self.gazeFile,
            zmpFile = self.zmpFile,
            comZ = self.comZ,
            trace = motion.trace,
            postureFeature = motion.postureFeature,
            postureTask = motion.postureTask)

        # FIXME: Not too good.
        self.feetFollower.comTask.controlGain.value = self.feetFollower.gain
        self.feetFollower.tasks['left-ankle'].controlGain.value = self.feetFollower.gain
        self.feetFollower.tasks['right-ankle'].controlGain.value = self.feetFollower.gain
        self.feetFollower.postureTask.controlGain.value = self.feetFollower.gain
        self.feetFollower.tasks['waist'].controlGain.value = self.feetFollower.gain

        unlockedDofsRleg = []
        unlockedDofsLleg = []
        for i in xrange(6):
            #FIXME: HRP-2 specific
            unlockedDofsRleg.append(6 + i)
            unlockedDofsLleg.append(6 + 6 + i)


        # Push the tasks into supervisor.
        motion.supervisor.addFeetFollowerStartCall(self.feetFollower.feetFollower.name,
                                                   self.interval[0])

        motion.supervisor.addTask(self.feetFollower.postureTask.name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 9,
                                  ())
        motion.supervisor.addTask(self.feetFollower.comTask.name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 3,
                                  ())
        motion.supervisor.addTask(self.feetFollower.tasks['left-ankle'].name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 2,
                                  tuple(unlockedDofsLleg))
        motion.supervisor.addTask(self.feetFollower.tasks['right-ankle'].name,
                                  self.interval[0], self.interval[1],
                                  self.priority + 1,
                                  tuple(unlockedDofsRleg))
        motion.supervisor.addTask(self.feetFollower.tasks['waist'].name,
                                  self.interval[0], self.interval[1],
                                  self.priority,
                                  ())

    def __str__(self):
        return "walking motion ({0} footstep(s))".format(len(self.footsteps))

    def setupTrace(self, trace):
        pass
