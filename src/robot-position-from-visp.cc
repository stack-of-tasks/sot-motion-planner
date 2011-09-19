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
#include "robot-position-from-visp.hh"
#include "time.hh"

RobotPositionFromVisp::RobotPositionFromVisp (const std::string& name)
  : dg::Entity (name),
    position_ (3),
    positionTimestamp_ (3),

    cMoIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "cMo")),
    cMoTimestampIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "cMoTimestamp")),

    plannedObjectPositionIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "plannedObjectPosition")),

    positionOut_ (INIT_SIGNAL_OUT
		  ("position", RobotPositionFromVisp::updatePosition, "Vector")),
    positionTimestampOut_
    (INIT_SIGNAL_OUT
     ("positionTimestamp", RobotPositionFromVisp::updatePositionTimestamp, "Vector"))
{
  signalRegistration (cMoIn_ << cMoTimestampIn_
		      << plannedObjectPositionIn_
		      << positionOut_
		      << positionTimestampOut_);
  positionOut_.setNeedUpdateFromAllChildren (true);
  positionTimestampOut_.setNeedUpdateFromAllChildren (true);
}

RobotPositionFromVisp::~RobotPositionFromVisp ()
{}

void
RobotPositionFromVisp::update (int t)
{
  const sot::MatrixHomogeneous& cMo = cMoIn_ (t);
  const sot::MatrixHomogeneous& wMo = plannedObjectPositionIn_ (t);

  // cMw = cMo * oMw = cMo * wMo^{-1}
  position_ = MatrixHomogeneousToXYTheta (wMo * cMo.inverse ());
  positionTimestamp_ = cMoTimestampIn_ (t);
}

ml::Vector&
RobotPositionFromVisp::updatePosition (ml::Vector& res, int t)
{
  update (t);
  res = position_;
  return res;
}

ml::Vector&
RobotPositionFromVisp::updatePositionTimestamp (ml::Vector& res, int t)
{
  update (t);
  res = positionTimestamp_;
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (RobotPositionFromVisp, "RobotPositionFromVisp");
