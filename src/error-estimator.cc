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

#include <dynamic-graph/command-setter.h>

#include "common.hh"
#include "error-estimator.hh"
#include "set-reference-trajectory.hh"

ErrorEstimator::ErrorEstimator (const std::string& name)
  : dg::Entity (name),
    worldTransformation_ (),
    position_ (dg::nullptr,
	       MAKE_SIGNAL_STRING (name, true, "Vector", "position")),
    error_ (INIT_SIGNAL_OUT ("error", ErrorEstimator::updateError, "Vector")),
    referenceTrajectory_ (dg::nullptr)
{
  signalRegistration (position_ << error_);

  error_.setNeedUpdateFromAllChildren (true);

  std::string docstring;
  addCommand ("setReferenceTrajectory",
	      new command::SetReferenceTrajectory<ErrorEstimator>
	      (*this, docstring));

  addCommand ("setWorldTransformation",
	      new dg::command::Setter<ErrorEstimator, ml::Matrix>
	      (*this, &ErrorEstimator::worldTransformation, docstring));
}

ErrorEstimator::~ErrorEstimator ()
{}

ml::Vector&
ErrorEstimator::updateError (ml::Vector& res, int t)
{
  if (res.size () != 3)
    res.resize (3);
  res.setZero ();

  if (!referenceTrajectory_)
    return res;

  referenceTrajectory_->update (t);
  
  ml::Vector com_ = referenceTrajectory_->comOut ();
  sot::MatrixHomogeneous com = XYThetaToMatrixHomogeneous (com_);

  // FIXME: waist position here.
  sot::MatrixHomogeneous planned = com;

  // FIXME: don't use the current one but synchronize to take
  // into account the delay in packet transmision.
  sot::MatrixHomogeneous estimated = worldTransformation_ *
    XYThetaToMatrixHomogeneous (position_ (t));

  sot::MatrixHomogeneous error = planned * estimated.inverse ();
  res = MatrixHomogeneousToXYTheta (error);
  return res;
}

void
ErrorEstimator::setReferenceTrajectory (FeetFollower* ptr)
{
  referenceTrajectory_ = ptr;
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (ErrorEstimator, "ErrorEstimator");
