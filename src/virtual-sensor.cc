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

VirtualSensor::VirtualSensor (const std::string& name)
  : dg::Entity (name),
    robotStateIn_ (dg::nullptr,
		   MAKE_SIGNAL_STRING (name, true, "Vector", "robotState")),
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
  signalRegistration (robotStateIn_
		      << expectedObstaclePositionIn_
		      << obstaclePositionIn_ 
		      << positionOut_
		      << positionTimestampOut_);
  positionOut_.setNeedUpdateFromAllChildren (true);
}

VirtualSensor::~VirtualSensor ()
{}

ml::Vector&
VirtualSensor::updatePosition (ml::Vector& res, int t)
{
  if (res.size () != 6)
    res.resize (6);
  res.setZero ();

  const ml::Vector& robotState = robotStateIn_ (t);
  const sot::MatrixHomogeneous& expectedObstaclePosition =
    expectedObstaclePositionIn_ (t);
  const sot::MatrixHomogeneous& obstaclePosition =
    obstaclePositionIn_ (t);

  ml::Vector pose (3);
  pose (0) = robotState (0);
  pose (1) = robotState (1);
  pose (2) = robotState (5);
  sot::MatrixHomogeneous robotPosition = XYThetaToMatrixHomogeneous(pose);

  sot::MatrixHomogeneous estimatedPosition =
    obstaclePosition * expectedObstaclePosition.inverse () * robotPosition;

  res = MatrixHomogeneousToXYTheta (estimatedPosition);
  return res;
}

ml::Vector&
VirtualSensor::updatePositionTimestamp (ml::Vector& res, int)
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

  typedef boost::posix_time::ptime ptime_t;

  if (res.size () != 2)
    res.resize (2);

  ptime_t time =
    boost::posix_time::microsec_clock::universal_time ();
  int64_t sec = time.time_of_day ().total_seconds();
  int64_t usec =
    time.time_of_day ().total_microseconds () - sec * 1000000;

  typedef boost::numeric::converter<double, int64_t> Int64_t2Double;

  res (0) = Int64_t2Double::convert (sec);
  res (1) = Int64_t2Double::convert (usec);
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (VirtualSensor, "VirtualSensor");
