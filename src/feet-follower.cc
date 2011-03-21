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

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

sot::MatrixHomogeneous
transformPgFrameIntoAnkleFrame (double tx, double ty, double tz, double theta,
				const sot::MatrixHomogeneous& feetToAnkle)
{
  ml::Vector t (3);
  t (0) = tx;
  t (1) = ty;
  t (2) = tz;

  sot::VectorRollPitchYaw vR;
  vR (2) = theta;
  sot::MatrixRotation R;
  vR.toMatrix (R);

  sot::MatrixHomogeneous tmp;
  tmp.buildFrom (R, t);
  sot::MatrixHomogeneous tmp2;
  tmp2 = feetToAnkle * tmp;
  return tmp2;
}


using ::dynamicgraph::command::Setter;

FeetFollower::FeetFollower (const std::string& name)
  : Entity(name),
    t_ (std::numeric_limits<int>::min ()),
    com_ (3),
    zmp_ (3),
    leftAnkle_ (),
    rightAnkle_ (),
    comZ_ (0.),
    leftFootToAnkle_ (),
    rightFootToAnkle_ (),
    initialLeftAnklePosition_ (),
    initialRightAnklePosition_ (),
    started_ (false),
    comOut_ (INIT_SIGNAL_OUT ("com", FeetFollower::updateCoM, "Vector")),
    zmpOut_ (INIT_SIGNAL_OUT ("zmp", FeetFollower::updateZmp, "Vector")),
    leftAnkleOut_
    (INIT_SIGNAL_OUT
     ("left-ankle", FeetFollower::updateLeftAnkle, "MatrixHomo")),
    rightAnkleOut_
    (INIT_SIGNAL_OUT
     ("right-ankle", FeetFollower::updateRightAnkle, "MatrixHomo"))
{
  signalRegistration (zmpOut_ << comOut_ << leftAnkleOut_ << rightAnkleOut_);

  zmpOut_.setNeedUpdateFromAllChildren (true);
  comOut_.setNeedUpdateFromAllChildren (true);
  leftAnkleOut_.setNeedUpdateFromAllChildren (true);
  rightAnkleOut_.setNeedUpdateFromAllChildren (true);

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






