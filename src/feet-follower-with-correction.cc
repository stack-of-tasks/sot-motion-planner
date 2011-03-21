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

#include "common.hh"
#include "feet-follower-with-correction.hh"

namespace ublas = boost::numeric::ublas;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeetFollowerWithCorrection,
				   "FeetFollowerWithCorrection");

FeetFollowerWithCorrection::FeetFollowerWithCorrection (const std::string& name)
  : FeetFollower (name),
    referenceTrajectory_ (),
    offsetIn_
    (dg::nullptr, MAKE_SIGNAL_STRING (name, true, "Vector", "offset")),
    correction_ ()
{
  leftAnkleOut_.addDependency (offsetIn_);
  rightAnkleOut_.addDependency (offsetIn_);
  comOut_.addDependency (offsetIn_);
  zmpOut_.addDependency (offsetIn_);

  signalRegistration (offsetIn_);
}

FeetFollowerWithCorrection::~FeetFollowerWithCorrection ()
{}

void
FeetFollowerWithCorrection::impl_update ()
{
  if (!referenceTrajectory_)
    return;

  if (started_)
    referenceTrajectory_->start ();

  updateCorrection ();

  referenceTrajectory_->updateLeftAnkle (leftAnkle_, t_);
  referenceTrajectory_->updateRightAnkle (rightAnkle_, t_);
  referenceTrajectory_->updateCoM (com_, t_);
  referenceTrajectory_->updateZmp (zmp_, t_);

  ml::Vector comH (4);
  ml::Vector zmpH (4);

  for (unsigned i = 0; i < 3; ++i)
    comH (i) = com_ (i), zmpH (i) = zmp_ (i);
  comH (3) = zmpH (3) = 1.;

  leftAnkle_ = correction_ * leftAnkle_;
  rightAnkle_ = correction_ * rightAnkle_;
  comH = correction_ * comH;
  zmpH = correction_ * zmpH;

  for (unsigned i = 0; i < 3; ++i)
    com_ (i) = comH (i), zmp_ (i) = zmpH (i);
}

namespace
{
  bool isDoubleSupportStart(const WalkMovement& movement, const double& t)
  {
    typedef std::pair<double, WalkMovement::SupportFoot> iter_t;
    BOOST_FOREACH (const iter_t& e, movement.supportFoot)
      {
	if (e.first > t)
	  return false;
	if (e.first == t)
	  return e.second == WalkMovement::SUPPORT_FOOT_DOUBLE;
      }
    return false;
  }
} // end of anonymous namespace.

void
FeetFollowerWithCorrection::updateCorrection ()
{
  // if (isDoubleSupportStart (referenceTrajectory.walkMovement ()))
  //   {
  //     // update position estimation
  //   }
}
