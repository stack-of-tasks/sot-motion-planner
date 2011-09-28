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

#include <dynamic-graph/command-setter.h>

#include "common.hh"
#include "robot-position-from-visp.hh"
#include "time.hh"

RobotPositionFromVisp::RobotPositionFromVisp (const std::string& name)
  : dg::Entity (name),
    cMc_ (),
    position_ (3),
    positionTimestamp_ (3),
    dbgcMo_ (),
    dbgPosition_ (),

    cMoIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "cMo")),
    cMoTimestampIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "cMoTimestamp")),

    plannedObjectPositionIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "plannedObjectPosition")),

    wMcIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "wMc")),
    wMrIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "wMr")),

    positionOut_ (INIT_SIGNAL_OUT
		  ("position",
		   RobotPositionFromVisp::updatePosition, "Vector")),
    positionTimestampOut_
    (INIT_SIGNAL_OUT
     ("positionTimestamp",
      RobotPositionFromVisp::updatePositionTimestamp, "Vector")),

    dbgcMoOut_ (INIT_SIGNAL_OUT
		("dbgcMo", RobotPositionFromVisp::updateDbgcMo, "MatrixHomo")),
    dbgPositionOut_ (INIT_SIGNAL_OUT
		     ("dbgPosition",
		      RobotPositionFromVisp::updateDbgPosition, "MatrixHomo"))

{
  signalRegistration (cMoIn_ << cMoTimestampIn_
		      << plannedObjectPositionIn_
		      << wMcIn_ << wMrIn_
		      << positionOut_
		      << positionTimestampOut_
		      << dbgcMoOut_
		      << dbgPositionOut_);
  positionOut_.setNeedUpdateFromAllChildren (true);
  positionTimestampOut_.setNeedUpdateFromAllChildren (true);
  dbgcMoOut_.setNeedUpdateFromAllChildren (true);
  dbgPositionOut_.setNeedUpdateFromAllChildren (true);

  std::string docstring;
  addCommand ("setSensorTransformation",
	      new dg::command::Setter<RobotPositionFromVisp, ml::Matrix>
	      (*this, &RobotPositionFromVisp::sensorTransformation, docstring));
}

RobotPositionFromVisp::~RobotPositionFromVisp ()
{}



void
RobotPositionFromVisp::update (int t)
{
  const sot::MatrixHomogeneous& cMo = cMoIn_ (t);
  sot::MatrixHomogeneous wMo = plannedObjectPositionIn_ (t);

  const sot::MatrixHomogeneous& wMc = wMcIn_ (t);
  const sot::MatrixHomogeneous& wMr = wMrIn_ (t);

  sot::MatrixHomogeneous robotMc = wMr.inverse () * wMc;

  dbgcMo_ = cMc_ * cMo * cMc_.inverse ();

  // wMrobot = wMo * oMc * cMrobot = wMo * cMo^{-1} * robotMc^{-1}
  dbgPosition_ = wMo * dbgcMo_.inverse () * robotMc.inverse ();

  position_ = MatrixHomogeneousToXYTheta (dbgPosition_);
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


sot::MatrixHomogeneous&
RobotPositionFromVisp::updateDbgcMo (sot::MatrixHomogeneous& res, int)
{
  res = dbgcMo_;
  return res;
}

sot::MatrixHomogeneous&
RobotPositionFromVisp::updateDbgPosition (sot::MatrixHomogeneous& res, int)
{
  res = dbgPosition_;
  return res;
}



DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (RobotPositionFromVisp,
				    "RobotPositionFromVisp");
