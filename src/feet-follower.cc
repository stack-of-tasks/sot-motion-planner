// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <limits>
#include <string>

#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <jrl/mathtools/angle.hh>
#include <jrl/mal/boost.hh>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include "common.hh"
#include "discretized-trajectory.hh"
#include "feet-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

static const double STEP = 0.005; //FIXME:

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

/// \brief Compute the ankle position in the world frame knowing the
/// foot position and the transformation from the foot to the ankle.
///
/// wMa = ankle position in world frame.
/// wMf = foot position in world frame.
/// fMa = ankle position in foot frame.
///
/// wMf is computed from (x,y,z,theta) information
///
/// wMa = wMf * fMa
///
sot::MatrixHomogeneous
computeAnklePositionInWorldFrame (double footX, double footY, double footZ, double footYaw,
				  const sot::MatrixHomogeneous& fMa)
{
  jrlMathTools::Angle theta (footYaw);

  ml::Vector xyztheta (4);
  xyztheta (0) = footX;
  xyztheta (1) = footY;
  xyztheta (2) = footZ;
  xyztheta (3) = theta.value ();

  sot::MatrixHomogeneous wMf =
    XYZThetaToMatrixHomogeneous (xyztheta);
  return wMf * fMa;
}

WalkMovement::WalkMovement
(const sot::DiscretizedTrajectory& leftFoot,
 const sot::DiscretizedTrajectory& rightFoot,
 const sot::DiscretizedTrajectory& com,
 const sot::DiscretizedTrajectory& zmp,
 const sot::DiscretizedTrajectory& waistYaw,
 const sot::DiscretizedTrajectory& waist,
 const sot::MatrixHomogeneous& wMw_traj)
  : leftFoot (leftFoot),
    rightFoot (rightFoot),
    com (com),
    zmp (zmp),
    waistYaw (waistYaw),
    waist (waist),
    wMw_traj (wMw_traj)
{}

using ::dynamicgraph::command::Setter;

FeetFollower::FeetFollower (const std::string& name)
  : Entity(name),
    t_ (std::numeric_limits<int>::min ()),
    com_ (3),
    zmp_ (3),
    waistYaw_ (),
    waist_ (),
    leftAnkle_ (),
    rightAnkle_ (),
    comVelocity_ (3),
    waistYawVelocity_ (1),
    leftAnkleVelocity_ (6),
    rightAnkleVelocity_ (6),
    comZ_ (0.),
    leftFootToAnkle_ (),
    rightFootToAnkle_ (),
    initialLeftAnklePosition_ (),
    initialRightAnklePosition_ (),
    started_ (false),
    startTime_ (-1.),
    comOut_ (INIT_SIGNAL_OUT ("com", FeetFollower::updateCoM, "Vector")),
    zmpOut_ (INIT_SIGNAL_OUT ("zmp", FeetFollower::updateZmp, "Vector")),
    waistYawOut_ (INIT_SIGNAL_OUT
		  ("waistYaw", FeetFollower::updateWaistYaw, "MatrixHomo")),
    waistOut_ (INIT_SIGNAL_OUT
	       ("waist", FeetFollower::updateWaist, "MatrixHomo")),
    leftAnkleOut_
    (INIT_SIGNAL_OUT
     ("left-ankle", FeetFollower::updateLeftAnkle, "MatrixHomo")),
    rightAnkleOut_
    (INIT_SIGNAL_OUT
     ("right-ankle", FeetFollower::updateRightAnkle, "MatrixHomo")),

    comVelocityOut_ (INIT_SIGNAL_OUT ("comVelocity",
				      FeetFollower::updateCoMVelocity,
				      "Vector")),
    waistYawVelocityOut_ (INIT_SIGNAL_OUT
			  ("waistYawVelocity",
			   FeetFollower::updateWaistYawVelocity, "MatrixHomo")),
    leftAnkleVelocityOut_
    (INIT_SIGNAL_OUT
     ("left-ankleVelocity",
      FeetFollower::updateLeftAnkleVelocity, "MatrixHomo")),
    rightAnkleVelocityOut_
    (INIT_SIGNAL_OUT
     ("right-ankleVelocity",
      FeetFollower::updateRightAnkleVelocity, "MatrixHomo"))
{
  signalRegistration (zmpOut_ << comOut_ << waistYawOut_ << waistOut_
		      << leftAnkleOut_ << rightAnkleOut_
		      << comVelocityOut_ << waistYawVelocityOut_
		      << leftAnkleVelocityOut_ << rightAnkleVelocityOut_);

  zmpOut_.setNeedUpdateFromAllChildren (true);
  comOut_.setNeedUpdateFromAllChildren (true);
  waistYawOut_.setNeedUpdateFromAllChildren (true);
  waistOut_.setNeedUpdateFromAllChildren (true);
  leftAnkleOut_.setNeedUpdateFromAllChildren (true);
  rightAnkleOut_.setNeedUpdateFromAllChildren (true);

  comVelocityOut_.setNeedUpdateFromAllChildren (true);
  waistYawVelocityOut_.setNeedUpdateFromAllChildren (true);
  leftAnkleVelocityOut_.setNeedUpdateFromAllChildren (true);
  rightAnkleVelocityOut_.setNeedUpdateFromAllChildren (true);

  std::string docstring;
  addCommand ("setComZ", new Setter<FeetFollower, double>
	      (*this, &FeetFollower::setComZ, docstring));

  addCommand ("setLeftFootToAnkle",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this, &FeetFollower::setLeftFootToAnkle, docstring));
  addCommand ("setRightFootToAnkle",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this, &FeetFollower::setRightFootToAnkle, docstring));

  addCommand ("setInitialLeftAnklePosition",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this, &FeetFollower::setInitialLeftAnklePosition, docstring));
  addCommand ("setInitialRightAnklePosition",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this,
	       &FeetFollower::setInitialRightAnklePosition, docstring));

  addCommand ("start", new command::Start (*this, docstring));
}

FeetFollower::~FeetFollower ()
{}

void
FeetFollower::start ()
{
  started_ = true;
  startTime_ = t_ * STEP;
  impl_start ();
}

void
FeetFollower::update (int t)
{
  if (t <= t_)
    return;
  t_ = t;
  impl_update ();
}

ml::Vector&
FeetFollower::updateCoM (ml::Vector& res, int t)
{
  if (t > t_)
    update (t);
  res = com_;
  return res;
}

ml::Vector&
FeetFollower::updateZmp (ml::Vector& res, int t)
{
  if (t > t_)
    update (t);
  res = zmp_;
  return res;
}

sot::MatrixHomogeneous&
FeetFollower::updateWaistYaw (sot::MatrixHomogeneous& res, int t)
{
  if (t > t_)
    update (t);
  res = waistYaw_;
  return res;
}

sot::MatrixHomogeneous&
FeetFollower::updateWaist (sot::MatrixHomogeneous& res, int t)
{
  if (t > t_)
    update (t);
  res = waist_;
  return res;
}


sot::MatrixHomogeneous&
FeetFollower::updateLeftAnkle (sot::MatrixHomogeneous& res, int t)
{
  if (t > t_)
    update (t);
  res = leftAnkle_;
  return res;
}

sot::MatrixHomogeneous&
FeetFollower::updateRightAnkle (sot::MatrixHomogeneous& res, int t)
{
  if (t > t_)
    update (t);
  res = rightAnkle_;
  return res;
}

ml::Vector&
FeetFollower::updateCoMVelocity (ml::Vector& res, int t)
{
  if (t > t_)
    update (t);
  res = comVelocity_;
  return res;
}

ml::Vector&
FeetFollower::updateWaistYawVelocity (ml::Vector& res, int t)
{
  if (t > t_)
    update (t);
  res = waistYawVelocity_;
  return res;
}

ml::Vector&
FeetFollower::updateLeftAnkleVelocity (ml::Vector& res, int t)
{
  if (t > t_)
    update (t);
  res = leftAnkleVelocity_;
  return res;
}

ml::Vector&
FeetFollower::updateRightAnkleVelocity (ml::Vector& res, int t)
{
  if (t > t_)
    update (t);
  res = rightAnkleVelocity_;
  return res;
}

double
FeetFollower::getTime () const
{
  return t_ * STEP;
}

double
FeetFollower::getTrajectoryTime () const
{
  if (!started_)
    return 0.;
  return getTime () - startTime_;
}

double
FeetFollower::startTime () const
{
  return startTime_;
}



namespace command
{
  Start::Start (FeetFollower& entity, const std::string& docstring)
    : Command (entity, std::vector<Value::Type> (), docstring)
  {}

  Value Start::doExecute()
  {
    FeetFollower& entity = static_cast<FeetFollower&>(owner ());
    entity.start ();
    return Value ();
  }
} // end of namespace command.
