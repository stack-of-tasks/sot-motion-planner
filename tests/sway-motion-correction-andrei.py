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

from dynamic_graph.sot.dynamics.tools import *
from dynamic_graph.sot.motion_planner.sway_motion_pg import *

# play
def play(maxIter = 4000, clt = None):
    print("started")

    t = 3
    # Main.
    #  Main loop
    for i in xrange(maxIter):
        smc.dbgE.recompute(t)
        smc.dbgCorrectedE.recompute(t)
        smc.dbgVelocityWithoutCorrection.recompute(t)
        smc.dbgcMoWithCorrection.recompute(t)

        if False:
            print("--- {0} ---".format(t))
            print("output velocity: {0}".format(smc.outputPgVelocity.value))
            print("E: {0}".format(smc.dbgE.value))
            print("corrected E: {0}".format(smc.dbgCorrectedE.value))
            print("vel w/o correction: {0}".format(smc.dbgVelocityWithoutCorrection.value))
            print("cMo w correction: {0}".format(smc.dbgcMoWithCorrection.value))

        if clt:
            clt.updateElementConfig(
                'hrp', r.smallToFull(r.device.state.value))

        r.device.increment(timeStep)
        t += 1

    print("dumping")
    tr.dump()
    tr.close()

play(clt = clt)
