// Copyright 2013 Olivier Stasse
// Gepetto, LAAS, CNRS
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
#include <jrl/mathtools/angle.hh>

#include <dynamic-graph/command-setter.h>


#include "common.hh"
#include "go-to-one-position.hh"
#include "time.hh"

GoToOnePosition::GoToOnePosition(const std::string &name)
  : dg::Entity(name),
    robotPosition_(3),
    targetPosition_(3),
    pgVelocity_(3),
    gains_(3),

    robotPositionIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING(name,true,"Vector","robotPosition")),
    
    targetPositionIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING(name,true,"Vector","targetPosition")),

    
    pgVelocityOut_
    (INIT_SIGNAL_OUT
     ("pgVelocity",
      GoToOnePosition::updatePgVelocity,"Vector"))
{
  signalRegistration( robotPositionIn_ <<
                      targetPositionIn_ <<
                      pgVelocityOut_ );
  pgVelocityOut_.setNeedUpdateFromAllChildren(true);

  std::string docstring;
  addCommand("setGains",
             new dg::command::Setter<GoToOnePosition,ml::Vector>
             (*this,
              &GoToOnePosition::setGains,
              docstring));

  for(unsigned int i=0;i<3;i++)
    {
      gains_(i) = 1.0;
      pgVelocity_(i)=0.0;
    }
}

GoToOnePosition::~GoToOnePosition()
{
}

void 
GoToOnePosition::update(int t)
{
  ml::Vector tmpPgVel(3);
  targetPosition_ = targetPositionIn_(t);
  robotPosition_ = robotPositionIn_(t);

  // Switch to homogeneous matrix representation.
  sot::MatrixHomogeneous wMtarget = XYThetaToMatrixHomogeneous(targetPosition_);
  sot::MatrixHomogeneous wMrobot = XYThetaToMatrixHomogeneous(robotPosition_);
  sot::MatrixHomogeneous targetMrobot = wMtarget.inverse() * wMrobot;
  tmpPgVel = MatrixHomogeneousToXYTheta(targetMrobot);

  for(unsigned int i=0;i<2;i++)
    {
      tmpPgVel(i) = gains_(i)*tmpPgVel(i);
      
      if (tmpPgVel(i)>0.1)
        tmpPgVel(i)=0.1;
      else if (tmpPgVel(i)<-0.1)
        tmpPgVel(i)=-0.1;
    }

  double c=tmpPgVel(2);
  double d= 2*M_PI- c;

  if (fabs(c)<fabs(d))
    tmpPgVel(2) = c;
  else
    tmpPgVel(2) = d;

  if (tmpPgVel(2)>0.03)
    tmpPgVel(2)=0.03;
  else if (tmpPgVel(2)<-0.03)
    tmpPgVel(2)=-0.03;

  pgVelocity_=tmpPgVel;    
}



ml::Vector&
GoToOnePosition::updatePgVelocity(ml::Vector & res, int t)
{
  update(t);
  res=pgVelocity_;
  return res;
}
    
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (GoToOnePosition,
				    "GoToOnePosition");
