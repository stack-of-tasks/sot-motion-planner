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
#include <boost/numeric/conversion/converter.hpp>

#include <jrl/mal/boost.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>

#include "common.hh"
#include "feet-follower-with-correction.hh"

namespace ublas = boost::numeric::ublas;

namespace command
{
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

  SetFootsteps::SetFootsteps
  (FeetFollowerWithCorrection& entity, const std::string& docstring)
    : Command
      (entity,
       boost::assign::list_of (Value::DOUBLE) (Value::VECTOR),
       docstring)
  {}

  Value SetFootsteps::doExecute ()
  {
    FeetFollowerWithCorrection& entity =
      static_cast<FeetFollowerWithCorrection&> (owner ());

    std::vector<Value> values = getParameterValues ();
    double time = values[0].value ();
    ml::Vector footsteps = values[1].value ();

    entity.footstepsTime () = time;
    entity.footsteps () = footsteps;
    return Value ();
  }

} // end of namespace command.

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeetFollowerWithCorrection,
				   "FeetFollowerWithCorrection");

FeetFollowerWithCorrection::FeetFollowerWithCorrection (const std::string& name)
  : FeetFollower (name),
    referenceTrajectory_ (),
    offsetIn_
    (dg::nullptr, MAKE_SIGNAL_STRING (name, true, "Vector", "offset")),
    positionIn_
    (dg::nullptr, MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "position")),
    correctionLeftAnkle_ (),
    correctionRightAnkle_ (),
    correctionCom_ (),
    corrections_ (),
    maxErrorX_ (0.1),
    maxErrorY_ (0.1),
    maxErrorTheta_ (3.14 / 12.),
    footstepsTime_ (),
    footsteps_ (),
    dbgFootstepsOut_ (INIT_SIGNAL_OUT
		      ("dbgFootsteps",
		       FeetFollowerWithCorrection::updateDbgFootsteps,
		       "Vector"))

{
  dbgFootstepsOut_.setNeedUpdateFromAllChildren (true);

  signalRegistration (offsetIn_ << positionIn_ << dbgFootstepsOut_);

  std::string docstring = "";
  addCommand ("setReferenceTrajectory",
	      new command::SetReferenceTrajectory<FeetFollowerWithCorrection>
	      (*this, docstring));

  addCommand ("setSafetyLimits",
	      new command::SetSafetyLimits (*this, docstring));

  addCommand ("setFootsteps",
	      new command::SetFootsteps (*this, docstring));
}

FeetFollowerWithCorrection::~FeetFollowerWithCorrection ()
{}

ml::Vector&
FeetFollowerWithCorrection::updateDbgFootsteps (ml::Vector& res, int)
{
  res = footsteps_;
  return res;
}

void
FeetFollowerWithCorrection::updateVelocities ()
{
  //FIXME: for now just forward velocity without correction.
  referenceTrajectory_->updateLeftAnkleVelocity (leftAnkleVelocity_, t_);
  referenceTrajectory_->updateRightAnkleVelocity (rightAnkleVelocity_, t_);
  referenceTrajectory_->updateCoMVelocity (comVelocity_, t_);
  referenceTrajectory_->updateWaistYawVelocity (waistYawVelocity_, t_);
}

void
FeetFollowerWithCorrection::impl_update ()
{
  if (!referenceTrajectory_)
    return;

  referenceTrajectory_->updateLeftAnkle (leftAnkle_, t_);
  referenceTrajectory_->updateRightAnkle (rightAnkle_, t_);
  referenceTrajectory_->updateCoM (com_, t_);
  referenceTrajectory_->updateZmp (zmp_, t_);
  referenceTrajectory_->updateWaistYaw (waistYaw_, t_);
  referenceTrajectory_->updatePosture (posture_, t_);

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


  jrlMathTools::Angle theta
    (jrlMathTools::Angle(MatrixHomogeneousToXYTheta (waistYaw_) (2)) +
     jrlMathTools::Angle(MatrixHomogeneousToXYTheta (correctionCom_) (2)));

  ml::Vector xytheta (3);
  xytheta (0) = xytheta (1) = 0.;
  xytheta (2) = theta.value ();
  waistYaw_ = XYThetaToMatrixHomogeneous (xytheta);

  updateVelocities ();
}

void
FeetFollowerWithCorrection::impl_start ()
{
  std::cout
    << "/!\\ feet follower with correction started /!\\"
    << std::endl;
  if (!referenceTrajectory_)
    return;
  referenceTrajectory_->start ();

  computeNewCorrection ();
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
	if (t >= e.first && std::fabs (e.first - t) <= 1e-2)
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

  bool before (const double& t, const sot::ErrorTrajectory& trajectory)
  {
    return t < trajectory.getLowerBound (trajectory.getRange ());
  }

  bool after (const double& t, const sot::ErrorTrajectory& trajectory)
  {
    return t > trajectory.getUpperBound (trajectory.getRange ());
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
	if (time >= iter->first
	    && std::fabs (time - iter->first) <= 1e-2)
	  return iter;
      }
    return supportFoot.end ();
  }

  // If the correction goes toward left, do it with left foot first,
  // otherwise do it with right foot first.
  //
  // Here we abandon current correction and will retry at the
  // end of the next step, it leftFirst is true, next time it will
  // be false.
  bool shouldDelayCorrection (bool leftFirst,
			      boost::shared_ptr<Correction> correction,
			      const sot::MatrixHomogeneous& previousCorrection,
			      const sot::MatrixHomogeneous& leftAnkle,
			      const sot::MatrixHomogeneous& rightAnkle,
			      const FeetFollower* referenceTrajectory,
			      const double& t)
  {
    return false; //FIXME:

    double nextStepTime = leftFirst ?
      correction->leftAnkleCorrection.getRange ().second
      : correction->rightAnkleCorrection.getRange ().second;

    const sot::Trajectory::vector_t& foot =
      leftFirst ?
      (*referenceTrajectory->walkMovement ()->leftFoot) (nextStepTime)
      : (*referenceTrajectory->walkMovement ()->rightFoot) (nextStepTime);

    // Next ankle position (can be left or right)
    //
    // {}^{w}M_{ankle} = {}^{w}M_{w_{traj}} * {}^{w_{traj}}M_{ankle}
    sot::MatrixHomogeneous ankle =
      referenceTrajectory->walkMovement ()->wMw_traj *
      computeAnklePositionInWorldFrame
      (foot[0], foot[1], foot[2], foot[3],
       leftFirst ?
       referenceTrajectory->leftFootToAnkle ()
       : referenceTrajectory->rightFootToAnkle ());

    ankle =
      XYThetaToMatrixHomogeneous
      (leftFirst ?
       correction->leftAnkleCorrection (nextStepTime)
       : correction->rightAnkleCorrection (nextStepTime))
      * previousCorrection
      * ankle;

    // compute the new ankle position in the frame of the
    // old ankle position.
    ankle = ankle
      * (leftFirst ? leftAnkle.inverse () : rightAnkle.inverse ());

    if (leftFirst && ankle (1, 3) < -1e-8)
      {
	std::cout << "Delaying correction at t = " << t
		  << ", " << (leftFirst ? 'l' : 'r') << std::endl
		  << "\t (dx,dy) = ("
		  << ankle (0, 3) << ","
		  << ankle (1, 3) << ","
		  << ")" << std::endl;
	return true;
      }
    if (!leftFirst && ankle (1, 3) > 1e-8)
      {
	std::cout << "Delaying correction at t = " << t
		  << ", " << (leftFirst ? 'l' : 'r') << std::endl;
	return true;
      }
    return false;
  }
} // end of anonymous namespace.

void
FeetFollowerWithCorrection::computeNewCorrection ()
{
  const double time = referenceTrajectory_->getTrajectoryTime ();

  if (time > 0.
      && (!isDoubleSupportStart (*referenceTrajectory_->walkMovement (), time)
	  || !correctionIsFinished (corrections_, time)))
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
  double firstEpsilon = 0.2 * (t2->first - t1->first);
  double secondEpsilon = 0.2 * (t3->first - t4->first);

  sot::ErrorTrajectory::interval_t firstInterval =
    sot::ErrorTrajectory::makeInterval
    (t1->first + firstEpsilon, t2->first - firstEpsilon);
  sot::ErrorTrajectory::interval_t secondInterval =
    sot::ErrorTrajectory::makeInterval
    (t3->first + secondEpsilon, t4->first - secondEpsilon);

  // double start = t1->first + 0.15 + 0.95;
  // double end = t1->first + 0.15 + 1.05;

  double start = t1->first;
  double end = t4->first;
  double eps = 0.2 * (end - start);

  sot::ErrorTrajectory::interval_t comCorrectionInterval =
    sot::ErrorTrajectory::makeInterval
    (start + eps, end - eps);

  ml::Vector tmp (3);
  tmp.setZero ();
  if (offsetIn_.access (t_).size () == 3)
    tmp = offsetIn_.access (t_);

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

  // Express the error in the world frame instead
  // of the waist frame.
  sot::MatrixHomogeneous p = positionIn_.access (t_);
  sot::ErrorTrajectory::vector_t errorW =
    MatrixHomogeneousToXYTheta
    (p
     * XYThetaToMatrixHomogeneous (error)
     * p.inverse ()).accessToMotherLib ();

  sot::MatrixHomogeneous previousCorrection;
  sot::MatrixHomogeneous positionError;

  previousCorrection.setIdentity ();
  positionError.setIdentity ();

  if (!corrections_.empty () && corrections_[corrections_.size () - 1])
    previousCorrection =
      corrections_[corrections_.size () - 1]->positionError;

  positionError = XYThetaToMatrixHomogeneous (errorW) * previousCorrection;

  boost::shared_ptr<Correction> correction =
    boost::make_shared<Correction>
    (positionError,
     leftFirst ? firstInterval  : secondInterval,
     leftFirst ? secondInterval : firstInterval,
     comCorrectionInterval, errorW);

  if (shouldDelayCorrection (leftFirst,
			     correction,
			     previousCorrection,
			     leftAnkle_,
			     rightAnkle_,
			     referenceTrajectory_, t_))
    return;
  corrections_.push_back (correction);

  updateFootsteps (time, XYThetaToMatrixHomogeneous (errorW));

  std::cout
    << "Adding new correction, offset = " << error
    << ", t = " << t_
    << ", " << (leftFirst ? 'l' : 'r') << std::endl;
}

void
FeetFollowerWithCorrection::updateFootsteps
(const double& time, const sot::MatrixHomogeneous& error)
{
  ml::Vector xytheta (3);

  typedef boost::numeric::converter<unsigned, double> Double2Unsigned;

  unsigned modifiedStep =
    Double2Unsigned::convert
    (std::floor(time / footstepsTime_));
  if (modifiedStep > 0)
    modifiedStep += 1;

  for (unsigned i = 0; i < footsteps_.size (); i += 3)
    {
      if (i / 3 == modifiedStep)
	{
	  for (unsigned j = 0; j < 3; ++j)
	    xytheta (j) = footsteps_ (i + j);
	  sot::MatrixHomogeneous step = XYThetaToMatrixHomogeneous (xytheta);

	  step = error * step;
	  xytheta = MatrixHomogeneousToXYTheta (step);

	  for (unsigned j = 0; j < 3; ++j)
	    footsteps_ (i + j) = xytheta (j);
	}
    }
}

void
FeetFollowerWithCorrection::updateCorrection ()
{
  correctionLeftAnkle_.setIdentity ();
  correctionRightAnkle_.setIdentity ();
  correctionCom_.setIdentity ();

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

  if (corrections_.size () > 1 && corrections_[corrections_.size () - 2])
    previousCorrection = corrections_[corrections_.size () - 2]->positionError;
  else
    previousCorrection.setIdentity ();

  if (before (time, currentCorrection.leftAnkleCorrection))
    correctionLeftAnkle_ = previousCorrection;
  else if (contain (time, currentCorrection.leftAnkleCorrection))
    correctionLeftAnkle_ = XYThetaToMatrixHomogeneous
      (currentCorrection.leftAnkleCorrection (time)) * previousCorrection;
  else if (after (time, currentCorrection.leftAnkleCorrection))
    correctionLeftAnkle_ = currentCorrection.positionError;

  if (before (time, currentCorrection.rightAnkleCorrection))
    correctionRightAnkle_ = previousCorrection;
  else if (contain (time, currentCorrection.rightAnkleCorrection))
    correctionRightAnkle_ = XYThetaToMatrixHomogeneous
      (currentCorrection.rightAnkleCorrection (time)) * previousCorrection;
  else
    correctionRightAnkle_ = currentCorrection.positionError;

  if (before (time, currentCorrection.comCorrection))
    correctionCom_ = previousCorrection;
  else if (contain (time, currentCorrection.comCorrection))
    correctionCom_ = XYThetaToMatrixHomogeneous
      (currentCorrection.comCorrection (time)) * previousCorrection;
  else
    correctionCom_ = currentCorrection.positionError;
}

void
FeetFollowerWithCorrection::setReferenceTrajectory (FeetFollower* ptr)
{
  referenceTrajectory_ = ptr;
}
