# Copyright 2011, Thomas Moulard (CNRS/LAAS)
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

# Environment objects.
INSTALL(
  FILES
  sot-motion-planner/object/blueball.py
  sot-motion-planner/object/chair.py
  sot-motion-planner/object/config.py
  sot-motion-planner/object/coord.py
  sot-motion-planner/object/disk.py
  sot-motion-planner/object/disk2.py
  sot-motion-planner/object/floor.py
  sot-motion-planner/object/left-footstep.py
  sot-motion-planner/object/light.py
  sot-motion-planner/object/right-footstep.py
  sot-motion-planner/object/shelf.py
  sot-motion-planner/object/stand.py
  sot-motion-planner/object/table.py
  DESTINATION share/sot-motion-planner/object)

# Motion plans.
INSTALL(
  FILES
  sot-motion-planner/plan/demo-icra.yaml
  sot-motion-planner/plan/demo-icra2.yaml
  sot-motion-planner/plan/demo-icra3.yaml
  sot-motion-planner/plan/nothing.yaml
  sot-motion-planner/plan/reach-left-com.yaml
  sot-motion-planner/plan/reach-left.yaml
  sot-motion-planner/plan/reach-right-left-com.yaml
  sot-motion-planner/plan/reach-right-left.yaml
  sot-motion-planner/plan/turn-in-place-mocap.yaml
  sot-motion-planner/plan/walk-forward-virtual-sensor.yaml
  sot-motion-planner/plan/walk-in-place-and-reach.yaml
  sot-motion-planner/plan/walk-in-place-mocap-head.yaml
  sot-motion-planner/plan/walk-in-place-mocap.yaml
  sot-motion-planner/plan/walk-in-place-virtual-sensor.yaml
  sot-motion-planner/plan/walk-in-place-visp.yaml
  sot-motion-planner/plan/walk-in-place.yaml
  DESTINATION share/sot-motion-planner/plan)

# Reference trajectories.
INSTALL(
  FILES
  sot-motion-planner/trajectory/walk-forward-virtual-sensor/waist.dat
  DESTINATION
  share/sot-motion-planner/trajectory/walk-forward-virtual-sensor)

INSTALL(
  FILES
  sot-motion-planner/trajectory/walk-in-place/waist.dat
  sot-motion-planner/trajectory/walk-in-place/gaze.dat
  DESTINATION
  share/sot-motion-planner/trajectory/walk-in-place)

INSTALL(
  FILES
  sot-motion-planner/trajectory/walk-in-place-mocap-head/waist.dat
  sot-motion-planner/trajectory/walk-in-place-mocap-head/gaze.dat
  sot-motion-planner/trajectory/walk-in-place-mocap-head/zmp.dat
  DESTINATION
  share/sot-motion-planner/trajectory/walk-in-place-mocap-head)
