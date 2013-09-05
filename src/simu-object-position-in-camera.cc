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
#include "simu-object-position-in-camera.hh"
#include "time.hh"

SimuObjectPositionInCamera::SimuObjectPositionInCamera 
(const std::string& name)
  : dg::Entity (name),
    wpgMoinit_ (),
    camMo_ (),
    wMcam_ (),
    wMcom_ (),
    wpgMcom_ (),

    wpgMoinitIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "wpgMoinit")),

    wMcamIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "wMcam")),

    wMcomIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "wMcom")),

    wpgMcomIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "wpgMcom")),

    wpgMcomAttIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "wpgMcomAtt")),

    camMoOut_
    (INIT_SIGNAL_OUT
     ("camMo",
      SimuObjectPositionInCamera::updateCamMo,"MatrixHomo"))
{
  signalRegistration (wpgMoinitIn_<< wMcamIn_ << wMcomIn_ << wpgMcomIn_ 
                      << wpgMcomAttIn_ << camMoOut_ );
  camMoOut_.addDependency(wpgMcomIn_);
  camMoOut_.addDependency(wpgMcomAttIn_);
}


SimuObjectPositionInCamera::~SimuObjectPositionInCamera()
{}

void
SimuObjectPositionInCamera::update(int t)
{
  const sot::MatrixHomogeneous& wpgMoinit = wpgMoinit_ = wpgMoinitIn_ (t);
  const sot::MatrixHomogeneous& wMcam = wMcam_ = wMcamIn_ (t);
  const ml::Vector & wMcom = wMcomIn_(t);
  for(unsigned int i=0;i<3;i++)
    wMcom_(i,3)=wMcom(i);

  const ml::Vector & wpgMcomAtt = wpgMcomAttIn_(t);
  double psi = wpgMcomAtt(0), theta = wpgMcomAtt(1), phi=wpgMcomAtt(2);
  double cpsi = cos(psi); double spsi = sin(psi);
  double cthe = cos(theta); double sthe = sin(theta);
  double cphi = cos(phi); double sphi=sin(phi);

  wpgMcom_(0,0) = cthe * cphi; wpgMcom_(0,1) = spsi * sthe * cphi - cpsi * sphi; wpgMcom_(0,2) = cpsi * sthe * cphi + spsi*sphi;
  wpgMcom_(1,0) = cthe * sphi; wpgMcom_(1,1) = spsi * sthe * sphi + cpsi * cphi; wpgMcom_(1,2) = cpsi * sthe * sphi - spsi*cphi;
  wpgMcom_(2,0) = -sthe;       wpgMcom_(2,1) = spsi * cthe;                      wpgMcom_(2,2) = cpsi * cthe;

  for(unsigned int i=0;i<3;i++)
    for(unsigned int j=0;j<3;j++)
      wMcom_(i,j)=wpgMcom_(i,j);

  camMo_ = wMcam.inverse() * wMcom_ * wpgMcom_.inverse() * wpgMoinit;
}

void SimuObjectPositionInCamera::display(std::ostream &os) const
{
  dg::Entity::display(os);
  os << "wpgMoinit:" << wpgMoinit_ << std::endl;
  os << "wMcam" << wMcam_ << std::endl;
  os << "wMcom" << wMcom_ << std::endl;
  os << "wpgMcom" << wpgMcom_ << std:: endl;
}

sot::MatrixHomogeneous&
SimuObjectPositionInCamera::updateCamMo (sot::MatrixHomogeneous& res, int t)
{
  update(t);
  res = camMo_;
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (SimuObjectPositionInCamera,
				    "SimuObjectPositionInCamera");
