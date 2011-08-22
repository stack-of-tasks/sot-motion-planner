// Copyright 2011, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of sot-motion-planner.
// sot-motion-planner is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-motion-planner is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

#include <cmath>

#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/date.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/numeric/conversion/converter.hpp>

#include <jrl/mathtools/angle.hh>

#include "common.hh"
#include "virtual-sensor.hh"
#include "time.hh"

VirtualSensor::VirtualSensor (const std::string& name)
  : dg::Entity (name),
    expectedRobotPositionIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "expectedRobotPosition")),
    robotPositionIn_ (dg::nullptr,
		      MAKE_SIGNAL_STRING (name, true, "Vector", "robotPosition")),

    expectedObstaclePositionIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo",
			 "expectedObstaclePosition")),

    obstaclePositionIn_ (dg::nullptr,
			 MAKE_SIGNAL_STRING (name, true, "MatrixHomo",
					     "obstaclePosition")),
    positionOut_ (INIT_SIGNAL_OUT
		  ("position", VirtualSensor::updatePosition, "Vector")),
    positionTimestampOut_
    (INIT_SIGNAL_OUT
     ("positionTimestamp", VirtualSensor::updatePositionTimestamp, "Vector"))
{
  signalRegistration (expectedRobotPositionIn_
		      << robotPositionIn_
		      << expectedObstaclePositionIn_
		      << obstaclePositionIn_
		      << positionOut_
		      << positionTimestampOut_);
  positionOut_.setNeedUpdateFromAllChildren (true);
  positionTimestampOut_.setNeedUpdateFromAllChildren (true);
}

VirtualSensor::~VirtualSensor ()
{}

ml::Vector&
VirtualSensor::updatePosition (ml::Vector& res, int t)
{
  if (res.size () != 3)
    res.resize (3);
  res.setZero ();

  const sot::MatrixHomogeneous& expectedRobotPosition =
    expectedRobotPositionIn_ (t);
  const sot::MatrixHomogeneous& robotPosition = robotPositionIn_ (t);
  const sot::MatrixHomogeneous& expectedObstaclePosition =
    expectedObstaclePositionIn_ (t);
  const sot::MatrixHomogeneous& obstaclePosition =
    obstaclePositionIn_ (t);

  sot::MatrixHomogeneous estimatedPosition =
    robotPosition
    * obstaclePosition.inverse ()
    * expectedObstaclePosition
    * expectedRobotPosition.inverse ();

  res = MatrixHomogeneousToXYTheta (estimatedPosition);
  return res;
}

ml::Vector&
VirtualSensor::updatePositionTimestamp (ml::Vector& res, int)
{
  sot::motionPlanner::timestamp (res);
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (VirtualSensor, "VirtualSensor");
