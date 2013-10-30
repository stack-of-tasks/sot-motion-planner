// Copyright 2011
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

#include <boost/foreach.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpAdaptiveGain.h>

#include "object-position-kalman.hh"
#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>

//const std::string CMoKalman::CLASS_NAME="CMoKalman";
static const double STEP = 0.005; // Computation period

using namespace std;

vpHomogeneousMatrix
convertsotMHTovpMH(sot::MatrixHomogeneous src)
{
  vpHomogeneousMatrix dst;
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      dst[i][j] = src (i, j);
  return dst;
}
 //CMoKalman class 
CMoKalman::CMoKalman (const std::string& name)
  : dg::Entity (name),
    initialized_ (false),
    inputdcom_ (dg::nullptr,
		      MAKE_SIGNAL_STRING (name, true, "vector", "inputdcom")), 
    cMo_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "cMo")),
    wMc_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "wMc")),
    wMwaist_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "wMwaist")),
    simu_cMo_ (INIT_SIGNAL_OUT
		       ("simu_cMo",
			CMoKalman::update, "MatrixHomo")),
    FT_ (vpFeatureTranslation::cdMc),
    FThU_ (vpFeatureThetaU::cdRc),
    task_ (),
    X1_(3),
    X2_(3),
    W_(3,3),
    V_(3,3),
    P_(3,3),
    K_(3,3),
    Y_(3),
    Zi_(0.)
 {
  signalRegistration (cMo_ << wMc_ << wMwaist_ << inputdcom_  << simu_cMo_ );
  
  P_.eye(3);
  V_.eye(3);
  W_.eye(3);
  P_ = 10.*P_; // FIXME
  V_ = 100.*V_;
  W_ = 0.01*W_; // FIXME

  initialized_ = false;

  std::string docstring;
  addCommand
    ("initialize",
     new command::cMoKalman::Initialize
     (*this, docstring));

  addCommand
    ("setKalman",
     new command::cMoKalman::SetKalman
     (*this, docstring));

  simu_cMo_.addDependency ( wMc_ );
  simu_cMo_.addDependency ( cMo_ );
  simu_cMo_.addDependency ( inputdcom_ );
  simu_cMo_.addDependency ( wMwaist_ );
}     
     
CMoKalman::~CMoKalman ()
{
}

void
CMoKalman::initialize (int t)
{
	vpHomogeneousMatrix waistTc = convertsotMHTovpMH ( wMwaist_(t).inverse() * wMc_(t) );
	for ( int i=0 ; i<3 ; i++ ) { waistTc[i][3] = 0; }
	vpHomogeneousMatrix cwaistMo = waistTc * convertsotMHTovpMH (cMo_(t));
 	FT_.buildFrom(cwaistMo);
 	FThU_.buildFrom(cwaistMo);
  	task_.addFeature (FT_);
  	task_.addFeature (FThU_);
  	task_.computeControlLaw ();
 	X1_[0] = task_.error[0];
  	X1_[1] = task_.error[1];
  	X1_[2] = task_.error[5];
  	X2_ = X1_;
	Zi_ = cwaistMo[2][3];
}

sot::MatrixHomogeneous&
CMoKalman::update(sot::MatrixHomogeneous& ret, int t) {
	// Add protection if no initialization

  // Kalman Filter
   /// get velocity in the waist frame
	  vpColVector ComVel (4);
  for (unsigned i = 0; i < 2; ++i) { ComVel[i] = inputdcom_ (t)(i); }
  ComVel[2] = 0;
  ComVel[3] = 1; 
  
  vpHomogeneousMatrix waistTw = convertsotMHTovpMH( wMwaist_ (t).inverse() );
  for ( int i=0 ; i<3 ; i++ ) { waistTw[i][3] = 0; }

  vpColVector HomoVelocity(4);
  HomoVelocity = waistTw * ComVel;
  
  vpColVector Velocity(3);
  Velocity[0] = HomoVelocity[0];
  Velocity[1] = HomoVelocity[1];
  Velocity[2] = inputdcom_(t)(2);

   /// get cMo from Visp and put it on the waist frame
  vpHomogeneousMatrix waistTc = convertsotMHTovpMH ( wMwaist_(t).inverse() * wMc_(t) );
  for ( int i=0 ; i<3 ; i++ ) { waistTc[i][3] = 0; }

  vpHomogeneousMatrix cwaistMo = waistTc * convertsotMHTovpMH(cMo_(t));

  FT_.buildFrom(cwaistMo);
  FThU_.buildFrom(cwaistMo);
  task_.addFeature (FT_);
  task_.addFeature (FThU_);
  task_.computeControlLaw ();

  Y_[0] = task_.error[0];
  Y_[1] = task_.error[1];
  Y_[2] = task_.error[5];

   /// Prediction
  X2_ = X1_ + STEP * Velocity;
  P_ = P_ + W_;
   
   /// Correction
  K_ = P_ * (P_ + V_).inverseByLU();
  X1_ = X2_ + K_ * (Y_ - X2_);
  vpMatrix I(3,3);
  I.eye(3);
  P_ = ( I - K_ ) * P_;

  // transform x,y,theta to homogeneousMatrix 
  vpHomogeneousMatrix cwaistMoFiltered( X1_[0], X1_[1], Zi_, 0, 0, X1_[2] );

  // put it on the camera frame
  vpHomogeneousMatrix cMoFiltered = waistTc.inverse() * cwaistMoFiltered;
 
  // convert to sot::homogeneousMatrix
  for ( int i=0 ; i<4 ; i++ ) 
    { 
      for ( int j=0 ; j<4 ; j++ ) 
        { 
          ret(i,j) = cMoFiltered[i][j];
      }
  }    
  
  return ret;
}

//ml::Vector&
//CMoKalman::update(ml::Vector& ret, int t) {

void
CMoKalman::setKalman(const double& p, const double& v, const double& w)
{
  P_.eye(3);
  P_ = p *P_;
  V_.eye(3);
  V_ = v *V_;
  W_.eye(3);
  W_ = w *W_;
}

 // Dynamic graph
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN
(CMoKalman, "CMoKalman");

namespace command
{
  namespace cMoKalman
  {
    Initialize::Initialize (CMoKalman& entity,
							const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::INT),
	 docstring)
    {}

    Value
    Initialize::doExecute ()
    {
      CMoKalman& entity =
	static_cast<CMoKalman&> (owner ());
	std::vector<Value> values = getParameterValues ();
	int t = values[0].value ();
      entity.initialize (t);
      return Value ();
    }

  SetKalman::SetKalman (CMoKalman& entity,
						const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::DOUBLE) (Value::DOUBLE) (Value::DOUBLE),
	 docstring)
    {}

    Value
    SetKalman::doExecute ()
    {
      CMoKalman& entity =
	static_cast<CMoKalman&> (owner ());

      std::vector<Value> values = getParameterValues ();
      double dx = values[0].value ();
      double dy = values[1].value ();
      double dtheta = values[2].value ();

      entity.setKalman(dx, dy, dtheta);
      return Value ();
    } 
  }
}
