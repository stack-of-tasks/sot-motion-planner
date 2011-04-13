// Copyright 2011, Thomas Moulard, CNRS.
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

#include <cassert>
#include <algorithm>
#include <string>
#include <fstream>

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <jrl/mal/boost.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include "common.hh"
#include "feet-follower-with-correction.hh"

namespace ublas = boost::numeric::ublas;

namespace command
{
  SetReferenceTrajectory::SetReferenceTrajectory
  (FeetFollowerWithCorrection& entity, const std::string& docstring)
    : Command (entity, boost::assign::list_of (Value::STRING), docstring)
  {}

  Value SetReferenceTrajectory::doExecute ()
  {
    FeetFollowerWithCorrection& entity =
      static_cast<FeetFollowerWithCorrection&> (owner ());

    std::vector<Value> values = getParameterValues ();
    std::string name = values[0].value ();

    FeetFollower* referenceTrajectory = 0;
    if (dynamicgraph::g_pool.existEntity (name))
      {
	referenceTrajectory =
	  dynamic_cast<FeetFollower*> (&dynamicgraph::g_pool.getEntity (name));
	if (!referenceTrajectory)
	  std::cerr << "entity is not a FeetFollower" << std::endl;
      }
    else
      std::cerr << "invalid entity name" << std::endl;

    entity.setReferenceTrajectory (referenceTrajectory);
    return Value ();
  }

  SetSafetyLimits::SetSafetyLimits
  (FeetFollowerWithCorrection& entity, const std::string& docstring)
    : Command
      (entity,
       boost::assign::list_of (Value::DOUBLE) (Value::DOUBLE) (Value::DOUBLE),
       docstring)
  {}

  Value SetSafetyLimits::doExecute ()
  {
    FeetFollowerWithCorrection& entity =
      static_cast<FeetFollowerWithCorrection&> (owner ());

    std::vector<Value> values = getParameterValues ();
    double maxErrorX = values[0].value ();
    double maxErrorY = values[1].value ();
    double maxErrorTheta = values[2].value ();

    entity.setSafetyLimits (maxErrorX, maxErrorY, maxErrorTheta);
    return Value ();
  }

} // end of namespace command.

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeetFollowerWithCorrection,
				   "FeetFollowerWithCorrection");

sot::MatrixHomogeneous
XYThetaToMatrixHomogeneous (const sot::ErrorTrajectory::vector_t& xytheta)
{
  assert (xytheta.size () == 3);
  ml::Vector t (3);
  t (0) = xytheta[0];
  t (1) = xytheta[1];
  t (2) = 0.;

  sot::VectorRollPitchYaw vR;
  vR (2) = xytheta[2];
  sot::MatrixRotation R;
  vR.toMatrix (R);
  sot::MatrixHomogeneous res;
  res.buildFrom (R, t);
  return res;
}


FeetFollowerWithCorrection::FeetFollowerWithCorrection (const std::string& name)
  : FeetFollower (name),
    referenceTrajectory_ (),
    offsetIn_
    (dg::nullptr, MAKE_SIGNAL_STRING (name, true, "Vector", "offset")),
    correctionLeftAnkle_ (),
    correctionRightAnkle_ (),
    correctionCom_ (),
    corrections_ (),
    maxErrorX_ (0.1),
    maxErrorY_ (0.1),
    maxErrorTheta_ (3.14 / 12.)

{
  leftAnkleOut_.addDependency (offsetIn_);
  rightAnkleOut_.addDependency (offsetIn_);
  comOut_.addDependency (offsetIn_);
  zmpOut_.addDependency (offsetIn_);

  signalRegistration (offsetIn_);

  std::string docstring = "";
  addCommand ("setReferenceTrajectory",
	      new command::SetReferenceTrajectory (*this, docstring));

  addCommand ("setSafetyLimits",
	      new command::SetSafetyLimits (*this, docstring));
}

FeetFollowerWithCorrection::~FeetFollowerWithCorrection ()
{}

void
FeetFollowerWithCorrection::impl_update ()
{
  if (!referenceTrajectory_)
    return;

  referenceTrajectory_->updateLeftAnkle (leftAnkle_, t_);
  referenceTrajectory_->updateRightAnkle (rightAnkle_, t_);
  referenceTrajectory_->updateCoM (com_, t_);
  referenceTrajectory_->updateZmp (zmp_, t_);

  updateCorrection ();

  ml::Vector comH (4);
  ml::Vector zmpH (4);

  for (unsigned i = 0; i < 3; ++i)
    comH (i) = com_ (i), zmpH (i) = zmp_ (i);
  comH (3) = zmpH (3) = 1.;

  leftAnkle_ = correctionLeftAnkle_ * leftAnkle_;
  rightAnkle_ = correctionRightAnkle_ * rightAnkle_;
  comH = correctionCom_ * comH;

  // Correction is always the same for com and zmp.
  zmpH = correctionCom_ * zmpH;

  for (unsigned i = 0; i < 3; ++i)
    com_ (i) = comH (i), zmp_ (i) = zmpH (i);
}

void
FeetFollowerWithCorrection::impl_start ()
{
  if (!referenceTrajectory_)
    return;
  referenceTrajectory_->start ();
}

namespace
{
  bool phaseStart (const WalkMovement& movement,
		   const double& t,
		   WalkMovement::SupportFoot supportFoot)
  {
    typedef std::pair<double, WalkMovement::SupportFoot> iter_t;

    BOOST_FOREACH (const iter_t& e, movement.supportFoot)
      {
	if (e.first > t)
	  return false;
	if (std::fabs (e.first - t) < 1e-3)
	  return e.second == supportFoot;
      }
    return false;
  }

  bool isDoubleSupportStart (const WalkMovement& movement, const double& t)
  {
    return phaseStart (movement, t, WalkMovement::SUPPORT_FOOT_DOUBLE);
  }

  bool contain (const double& t, const sot::ErrorTrajectory& trajectory)
  {
    return (t >= trajectory.getLowerBound (trajectory.getRange ())
	    && t <= trajectory.getUpperBound (trajectory.getRange ()));
  }

  bool
  correctionIsFinished
  (const std::vector<boost::shared_ptr<Correction> >& corrections,
   const double& t)
  {
    if (corrections.empty ())
      return true;
    const Correction& correction = *corrections.back ();

    return t >= sot::ErrorTrajectory::getUpperBound
      (correction.leftAnkleCorrection.getRange ())
      && t >= sot::ErrorTrajectory::getUpperBound
      (correction.rightAnkleCorrection.getRange ())
      && t >= sot::ErrorTrajectory::getUpperBound
      (correction.comCorrection.getRange ());
  }

  WalkMovement::supportFoot_t::const_iterator
  findIter (const WalkMovement::supportFoot_t& supportFoot, const double& time)
  {
    WalkMovement::supportFoot_t::const_iterator iter = supportFoot.begin ();
    for (; iter != supportFoot.end (); ++iter)
      {
	if (iter->first == time)
	  return iter;
      }
    return supportFoot.end ();
  }
} // end of anonymous namespace.

void
FeetFollowerWithCorrection::computeNewCorrection ()
{
  const double time = referenceTrajectory_->getTrajectoryTime ();

  if (!isDoubleSupportStart (*referenceTrajectory_->walkMovement (), time)
      || !correctionIsFinished (corrections_, time))
    return;

  const WalkMovement::supportFoot_t& supportFoot =
    referenceTrajectory_->walkMovement ()->supportFoot;

  // Now - double support
  WalkMovement::supportFoot_t::const_iterator t = supportFoot.end ();
  // left or right
  WalkMovement::supportFoot_t::const_iterator t1 = supportFoot.end ();
  // double support
  WalkMovement::supportFoot_t::const_iterator t2 = supportFoot.end ();
  // left or right (opposite foot)
  WalkMovement::supportFoot_t::const_iterator t3 = supportFoot.end ();
  // double support
  WalkMovement::supportFoot_t::const_iterator t4 = supportFoot.end ();

  // Find next steps.
  t = findIter (supportFoot, time);
  if (t != supportFoot.end ())
    {
      t1 = t;
      ++t1;
      if (t1 != supportFoot.end ())
	{
	  t2 = t1;
	  ++t2;
	  if (t2 != supportFoot.end ())
	    {
	      t3 = t2;
	      ++t3;
	      if (t3 != supportFoot.end ())
		{
		  t4 = t3;
		  ++t4;
		}
	    }
	}
    }

  if (t == supportFoot.end () || t1 == supportFoot.end ()
      || t2 == supportFoot.end () || t3 == supportFoot.end ()
      || t4 == supportFoot.end ())
    return;

  assert (t->second == WalkMovement::SUPPORT_FOOT_DOUBLE);
  assert (t1->second == WalkMovement::SUPPORT_FOOT_LEFT
	  || t1->second == WalkMovement::SUPPORT_FOOT_RIGHT);
  assert (t2->second == WalkMovement::SUPPORT_FOOT_DOUBLE);
  assert (t3->second == WalkMovement::SUPPORT_FOOT_LEFT
	  || t3->second == WalkMovement::SUPPORT_FOOT_RIGHT);
  assert (t4->second == WalkMovement::SUPPORT_FOOT_DOUBLE);

  bool leftFirst = t1->second == WalkMovement::SUPPORT_FOOT_RIGHT;

  // Feet trajectories intervals.
  //
  // The correction must happen *in the air*, so to avoid troubles,
  // correction should not start exactly at the beginning of the
  // intervals (and should not stop at the end either).

  // 10%           - 80%        - 10%
  // no correction - correction - no correction
  double firstEpsilon = 0.1 * (t2->first - t1->first);
  double secondEpsilon = 0.1 * (t3->first - t4->first);

  sot::ErrorTrajectory::interval_t firstInterval =
    sot::ErrorTrajectory::makeInterval
    (t1->first + firstEpsilon, t2->first - firstEpsilon);
  sot::ErrorTrajectory::interval_t secondInterval =
    sot::ErrorTrajectory::makeInterval
    (t3->first + secondEpsilon, t4->first - secondEpsilon);
  sot::ErrorTrajectory::interval_t comCorrectionInterval =
    sot::ErrorTrajectory::makeInterval
    (t1->first + firstEpsilon * 4., t2->first - firstEpsilon * 4.);

  ml::Vector tmp = offsetIn_.accessCopy ();
  sot::ErrorTrajectory::vector_t error;
  if (tmp.size () == 3)
    error = tmp.accessToMotherLib ();
  else
    {
      error.resize (3);
      error.clear ();
    }

  if (error[0] > 0.)
    error[0] = std::min (error[0], maxErrorX_);
  else
    error[0] = -std::min (-error[0], maxErrorX_);

  if (error[1] > 0.)
    error[1] = std::min (error[1], maxErrorY_);
  else
    error[1] = -std::min (-error[1], maxErrorY_);

  if (error[2] > 0.)
    error[2] = std::min (error[2], maxErrorTheta_);
  else
    error[2] = -std::min (-error[2], maxErrorTheta_);

  // If the correction goes toward left, do it with left foot first,
  // otherwise do it with right foot first.
  //
  // Here we abandon current correction and will retry at the
  // end of the next step, it leftFirst is true, next time it will
  // be false.
  if (leftFirst && error[1] < 0.)
    return;
  if (!leftFirst && error[1] > 0.)
    return;

  sot::MatrixHomogeneous previousCorrection;
  sot::MatrixHomogeneous positionError;

  if (corrections_.size () > 1 && corrections_[corrections_.size () - 1])
    previousCorrection =
      corrections_[corrections_.size () - 1]->positionError;
  positionError = XYThetaToMatrixHomogeneous (error) * previousCorrection;

  corrections_.push_back
    (boost::make_shared<Correction>
     (positionError,
      leftFirst ? firstInterval  : secondInterval,
      leftFirst ? secondInterval : firstInterval,
      comCorrectionInterval, error));
  //std::cout << "Adding new correction, offset = " << tmp << std::endl;
}

void
FeetFollowerWithCorrection::updateCorrection ()
{
  if (!referenceTrajectory_ || !started_)
    return;

  const double time = referenceTrajectory_->getTrajectoryTime ();

  computeNewCorrection ();

  if (corrections_.empty ())
    return;
  if (!corrections_.back ())
    return;

  // Compute corrections.
  const Correction& currentCorrection = *(corrections_.back ());

  sot::MatrixHomogeneous previousCorrection;

  if (corrections_.size () > 1 && corrections_[corrections_.size () - 1])
    previousCorrection = corrections_[corrections_.size () - 1]->positionError;

  if (contain (time, currentCorrection.leftAnkleCorrection))
    correctionLeftAnkle_ = XYThetaToMatrixHomogeneous
      (currentCorrection.leftAnkleCorrection (time)) * previousCorrection;
  if (contain (time, currentCorrection.rightAnkleCorrection))
    correctionRightAnkle_ = XYThetaToMatrixHomogeneous
      (currentCorrection.rightAnkleCorrection (time)) * previousCorrection;
  if (contain (time, currentCorrection.comCorrection))
    correctionCom_ = XYThetaToMatrixHomogeneous
      (currentCorrection.comCorrection (time)) * previousCorrection;

}

void
FeetFollowerWithCorrection::setReferenceTrajectory (FeetFollower* ptr)
{
  referenceTrajectory_ = ptr;
}
