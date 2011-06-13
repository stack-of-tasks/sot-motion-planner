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

static const double DEFAULT_TIME_STEP = 0.005; //FIXME:

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
 const sot::MatrixHomogeneous& wMw_traj)
  : leftFoot (leftFoot),
    rightFoot (rightFoot),
    com (com),
    zmp (zmp),
    waistYaw (waistYaw),
    wMw_traj (wMw_traj)
{}

using ::dynamicgraph::command::Setter;

FeetFollower::FeetFollower (const std::string& name)
  : Entity(name),
    t_ (std::numeric_limits<int>::min ()),
    com_ (3),
    zmp_ (3),
    waistYaw_ (),
    leftAnkle_ (),
    rightAnkle_ (),
    comZ_ (0.),
    leftFootToAnkle_ (),
    rightFootToAnkle_ (),
    initialLeftAnklePosition_ (),
    initialRightAnklePosition_ (),
    started_ (false),
    startTime_ (-1.),
    timeStep_(DEFAULT_TIME_STEP),
    comOut_ (INIT_SIGNAL_OUT ("com", FeetFollower::updateCoM, "Vector")),
    zmpOut_ (INIT_SIGNAL_OUT ("zmp", FeetFollower::updateZmp, "Vector")),
    waistYawOut_ (INIT_SIGNAL_OUT
		  ("waistYaw", FeetFollower::updateWaistYaw, "MatrixHomo")),
    leftAnkleOut_
    (INIT_SIGNAL_OUT
     ("left-ankle", FeetFollower::updateLeftAnkle, "MatrixHomo")),
    rightAnkleOut_
    (INIT_SIGNAL_OUT
     ("right-ankle", FeetFollower::updateRightAnkle, "MatrixHomo"))
{
  signalRegistration (zmpOut_ << comOut_ << waistYawOut_
		      << leftAnkleOut_ << rightAnkleOut_);

  zmpOut_.setNeedUpdateFromAllChildren (true);
  comOut_.setNeedUpdateFromAllChildren (true);
  waistYawOut_.setNeedUpdateFromAllChildren (true);
  leftAnkleOut_.setNeedUpdateFromAllChildren (true);
  rightAnkleOut_.setNeedUpdateFromAllChildren (true);

  std::string docstring;
  docstring =
    "    Set height of center of mass during walk\n"
    "    \n"
    "    Input: a floating point number\n";
  addCommand ("setComZ", new Setter<FeetFollower, double>
	      (*this, &FeetFollower::setComZ, docstring));

  docstring =
    "    Set position of ankle in left foot local frame.\n"
    "    \n"
    "    Input: a matrix homogeneous\n";
  addCommand ("setLeftFootToAnkle",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this, &FeetFollower::setLeftFootToAnkle, docstring));

  docstring =
    "    Set position of ankle in right foot local frame.\n"
    "    \n"
    "    Input: a matrix homogeneous\n";
  addCommand ("setRightFootToAnkle",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this, &FeetFollower::setRightFootToAnkle, docstring));

  docstring =
    "    Set initial position of left ankle\n"
    "    \n"
    "    Input: a matrix homogeneous\n";
  addCommand ("setInitialLeftAnklePosition",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this, &FeetFollower::setInitialLeftAnklePosition, docstring));

  docstring =
    "    Set initial position of right ankle\n"
    "    \n"
    "    Input: a matrix homogeneous\n";
  addCommand ("setInitialRightAnklePosition",
	      new Setter<FeetFollower, maal::boost::Matrix>
	      (*this,
	       &FeetFollower::setInitialRightAnklePosition, docstring));

  docstring =
    "    \n"
    "    Start walking motion"
    "    \n"
    "    No input_n";
  addCommand ("start", new command::Start (*this, docstring));
  docstring =
    "    Set time step of discretized trajectories\n"
    "    \n"
    "    Input: a floating point number\n"
    "      default value is 0.005\n";
  addCommand ("setTimeStep",
	      new Setter<FeetFollower, double>
	      (*this, &FeetFollower::setTimeStep, docstring));
}

FeetFollower::~FeetFollower ()
{}

void FeetFollower::setTimeStep(const double& inTimeStep)
{
  timeStep_ = inTimeStep;
}

void
FeetFollower::start ()
{
  started_ = true;
  startTime_ = t_ * timeStep_;
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

double
FeetFollower::getTime () const
{
  return t_ * timeStep_;
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
