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
#include "visp-point-projection.hh"
#include "time.hh"

VispPointProjection::VispPointProjection (const std::string& name)
  : dg::Entity (name),
    xy_ (2),
    z_ (0.),

    cMoIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "cMo")),
    cMoTimestampIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "cMoTimestamp")),

    xyOut_ (INIT_SIGNAL_OUT
		  ("xy",
		   VispPointProjection::updateXy, "Vector")),
    zOut_ (INIT_SIGNAL_OUT
		  ("Z",
		   VispPointProjection::updateZ, "double"))
{
  signalRegistration (cMoIn_<< cMoTimestampIn_ << xyOut_ << zOut_);
  xyOut_.setNeedUpdateFromAllChildren (true);
  zOut_.setNeedUpdateFromAllChildren (true);

  xy_.setZero ();
}

VispPointProjection::~VispPointProjection ()
{}

void
VispPointProjection::update (int t)
{
  const sot::MatrixHomogeneous& cMo = cMoIn_ (t);
  const double& X = cMo (0, 3);
  const double& Y = cMo (1, 3);
  const double& Z = cMo (2, 3);

  using boost::posix_time::seconds;

  boost::posix_time::ptime now =
    boost::posix_time::microsec_clock::universal_time ();
  boost::posix_time::ptime cMoTime =
    sot::motionPlanner::timestampToDateTime (cMoTimestampIn_ (t));

  // If z is near zero or cMo too old (tracking may have been
  // lost), set to zero to stop the movement.
  if (now - cMoTime > seconds (1) || std::fabs (Z) < 1e-6)
    {
      xy_ (0) = 0.;
      xy_ (1) = 0.;
      z_ = 1.;
      return;
    }
  xy_ (0) = X / Z;
  xy_ (1) = Y / Z;
  z_ = Z;
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (VispPointProjection,
				    "VispPointProjection");
