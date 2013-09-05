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
    ConvergenceReached_(false),
    robotPosition_(3),
    targetPosition_(3),
    pgVelocity_(3),
    gains_(3),
    limits_up_(3),
    limits_bottom_(3),

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

  addCommand("setUpperLimit",
             new dg::command::Setter<GoToOnePosition,ml::Vector>
             (*this,
              &GoToOnePosition::setUpperLimit,
              docstring));

  addCommand("setBottomLimit",
             new dg::command::Setter<GoToOnePosition,ml::Vector>
             (*this,
              &GoToOnePosition::setBottomLimit,
              docstring));

  addCommand("setConvergence",
             new dg::command::Setter<GoToOnePosition,std::string>
             (*this,
              &GoToOnePosition::setConvergence,
              docstring));

  for(unsigned int i=0;i<3;i++)
    {
      pgVelocity_(i)=0.0;
      gains_(i)=1.0;
    }
  limits_up_(0) = 0.1 ; limits_bottom_(0)=-0.1;
  limits_up_(1) = 0.03; limits_bottom_(1)=-0.03;
  limits_up_(2) = 0.03 ;limits_bottom_(2)=-0.03;
        
}

GoToOnePosition::~GoToOnePosition()
{
}

void 
GoToOnePosition::update(int t)
{
  if (ConvergenceReached_)
    return;

  ml::Vector tmpPgVel(3);
  targetPosition_ = targetPositionIn_(t);
  robotPosition_ = robotPositionIn_(t);

  // Switch to homogeneous matrix representation.
  sot::MatrixHomogeneous wMtarget = XYThetaToMatrixHomogeneous(targetPosition_);
  sot::MatrixHomogeneous wMrobot = XYThetaToMatrixHomogeneous(robotPosition_);
  sot::MatrixHomogeneous robotMtarget = wMrobot.inverse() * wMtarget;
  tmpPgVel = MatrixHomogeneousToXYTheta(robotMtarget);

  if (sqrt(tmpPgVel(0)*tmpPgVel(0)+tmpPgVel(1)*tmpPgVel(1))<0.15)
    {
      pgVelocity_(0)=0.0;
      pgVelocity_(1)=0.0;
      pgVelocity_(2)=0.0;
      ConvergenceReached_=true;
    }
  else
    {
      for(unsigned int i=0;i<2;i++)
        {
          tmpPgVel(i) = gains_(i)*tmpPgVel(i);
          
          if (tmpPgVel(i)>limits_up_(i))
            tmpPgVel(i)=limits_up_(i);
          else if (tmpPgVel(i)<limits_bottom_(i))
            tmpPgVel(i)=limits_bottom_(i);
        }
      
      double c=tmpPgVel(2);
      double d= 2*M_PI- c;
      
      if (fabs(c)<fabs(d))
        tmpPgVel(2) = c;
      else
        tmpPgVel(2) = d;
      
      if (tmpPgVel(2)>limits_up_(2))
        tmpPgVel(2)=limits_up_(2);
      else if (tmpPgVel(2)<limits_bottom_(2))
        tmpPgVel(2)=limits_bottom_(2);
      
      pgVelocity_=tmpPgVel;    
    }
}



ml::Vector&
GoToOnePosition::updatePgVelocity(ml::Vector & res, int t)
{
  update(t);
  res=pgVelocity_;
  return res;
}
/* --------------------------------------------------------------------- */
/* --- DISPLAY --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

void GoToOnePosition::
display_vector(std::string title,
               std::ostream& os, const ml::Vector &avector) const
{
  os << title << std::endl << "( ";
  for(unsigned int i=0;i<2;i++)
    os << avector(i)<< ",";
  os << avector(2) << ")" << std::endl;
}

void GoToOnePosition::
display( std::ostream& os ) const
{
  display_vector(std::string("Gains"),os,gains_);
  display_vector(std::string("Upper limits"),os,limits_up_);
  display_vector(std::string("Lower limits"),os,limits_bottom_);
  os << "ConvergenceReached_:" << std::endl;
  if (ConvergenceReached_)
    os << "true" ;
  else
    os << "false" ;
  os << std::endl;
}

std::ostream&
operator<< ( std::ostream& os,const GoToOnePosition& g21p )
{
  g21p.display(os);
  return os;
}
    
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (GoToOnePosition,
				    "GoToOnePosition");
