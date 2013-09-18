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


#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

static const double STEP = 0.005; // Computation period
static const double T = 1.6; // Walking period

vpHomogeneousMatrix
convert(sot::MatrixHomogeneous src)
{
  vpHomogeneousMatrix dst;
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      dst[i][j] = src (i, j);
  return dst;
}

vpHomogeneousMatrix
convert(ml::Matrix src)
{
  vpHomogeneousMatrix dst;
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      dst[i][j] = src (i, j);
  return dst;
}


struct TimedInteractionMatrix
{
  vpMatrix L;
  double timestamp;
  double velref[3];
};

class SwayMotionCorrection : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
 public:
  /// \brief Input vector signal.
  typedef dg::SignalPtr<ml::Vector, int> signalVectorIn_t;
  /// \brief Input homogeneous matrix signal.
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;
  /// \brief Input matrix signal.
  typedef dg::SignalPtr<ml::Matrix, int> signalMatrixIn_t;

  /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<ml::Vector, int> signalVectorOut_t;
  /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int>
  signalMatrixHomoOut_t;

  /// \name Constructor and destructor.
  /// \{
  explicit SwayMotionCorrection (const std::string& name);
  virtual ~SwayMotionCorrection ();
  /// \}

  void initialize (const vpHomogeneousMatrix& cdMo, int t);
  void stop ();

  void setMaximumVelocity(const double& dx, const double& dy, const double& dtheta)
  {
    vmax_[0] = dx;
    vmax_[1] = dy;
    vmax_[2] = dtheta;
  }

protected:
  /// \brief Compute camera velocity from (current) waist velocity.
  vpVelocityTwistMatrix fromCameraToWaistTwist (int t);

  /// \brief Make sure that the velocity stays lower than vmax.
  vpColVector velocitySaturation (const vpColVector& velocity);

  /// \brief Update PG velocity callback.
  ml::Vector& updateVelocity (ml::Vector& v, int);

  /// \brief Is the error lower enough to stop?
  bool shouldStop(vpHomogeneousMatrix Error, double ErrorMoy) const;

  /// \brief Is the control law started?
  bool initialized_;

  /// \brief Gain used to compute the control law.
  double lambda_;

  /// \brief Maximum CoM velocity (x, y, theta).
  vpColVector vmax_;

  /// \brief Set before starting computing control law.
  vpHomogeneousMatrix cdMo_;

  /// \brief Current desired position w.r.t to the current pose.
  vpHomogeneousMatrix cdMc_;
  
  /// \brief Translation feature handling position servoing.
  vpFeatureTranslation FT_;
  /// \brief Theta U feature handling orientation servoing.
  vpFeatureThetaU FThU_;

  /// \brief Task computing the control law.
  vpServo task_;
   
  /// \brief Input com velocity (signal).
  signalVectorIn_t inputdcom_;
  /// \brief Output pattern generator velocity (signal).
  signalVectorOut_t outputPgVelocity_;

  /// \brief c*Mc
  signalMatrixHomoIn_t cMo_;
  signalVectorIn_t cMoTimestamp_;

  /// \brief waist position w.r.t world frame.
  signalMatrixHomoIn_t wMwaist_;
  /// \brief Camera position w.r.t. world frame.
  signalMatrixHomoIn_t wMcamera_;

  /// \brief If error is lower than this threshold then stop.
  double minThreshold_;

  /// \brief Error accumulation.
  vpColVector E_;

  /// \brief FIXME
  vpColVector integralLbk_;
   
  /// \brief Tab of previous E
  double E[2][320];
  
  /// \brief increment of E
  int Einc; 
  
  /// \brief last outputPgVelocity
  vpColVector inputComVel;
  
  /// \brief mean of E_ on a period   
  vpColVector E_tT;
};

namespace command
{
  namespace swayMotionCorrection
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class Initialize : public Command
    {
    public:
      Initialize (SwayMotionCorrection& entity,
		  const std::string& docstring);
      virtual Value doExecute ();
    };

    class SetMaximumVelocity : public Command
    {
    public:
      SetMaximumVelocity (SwayMotionCorrection& entity,
			  const std::string& docstring);
      virtual Value doExecute ();
    };
  } // end of namespace swayMotionCorrection.
} // end of namespace command.


SwayMotionCorrection::SwayMotionCorrection (const std::string& name)
  : dg::Entity (name),
    initialized_ (false),
    lambda_ (0.6),
    vmax_ (3),
    cdMc_ (),
    FT_ (vpFeatureTranslation::cdMc),
    FThU_ (vpFeatureThetaU::cdRc),
    task_ (),
    inputdcom_ (dg::nullptr,
		      MAKE_SIGNAL_STRING (name, true, "vector", "inputdcom")), 
    outputPgVelocity_ (INIT_SIGNAL_OUT
		       ("outputPgVelocity",
			SwayMotionCorrection::updateVelocity, "Vector")),
    cMo_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "cMo")),
    cMoTimestamp_ (dg::nullptr,
		   MAKE_SIGNAL_STRING
		   (name, true, "Vector", "cMoTimestamp")),
    wMwaist_ (dg::nullptr,
	      MAKE_SIGNAL_STRING
	      (name, true, "MatrixHomo", "wMwaist")),
    wMcamera_ (dg::nullptr,
	       MAKE_SIGNAL_STRING
	       (name, true, "MatrixHomo", "wMcamera")),
    minThreshold_ (0.01),
    E_ (2),
    inputComVel(3),
    integralLbk_ (2),
    E_tT(2)
{
  signalRegistration (inputdcom_ << outputPgVelocity_ << cMo_ << cMoTimestamp_ 
		      << wMwaist_ << wMcamera_ );

    vmax_[0] = 0.25;
    vmax_[1] = 0.2;
    vmax_[2] = 0.2;
	
  task_.setServo (vpServo::EYEINHAND_CAMERA);
  task_.setInteractionMatrixType (vpServo::CURRENT);
  task_.setLambda (lambda_/2);

  std::string docstring;
  addCommand
    ("initialize",
     new command::swayMotionCorrection::Initialize
     (*this, docstring));

  addCommand
    ("setMaximumVelocity",
     new command::swayMotionCorrection::SetMaximumVelocity
     (*this, docstring));
     
     // Signal dependencies
  // outputPgVelocity_.addDependency ( inputdcom_ ); bug : loop between sway-motion and pg
   outputPgVelocity_.addDependency ( cMo_ );
   outputPgVelocity_.addDependency ( wMwaist_ );
   outputPgVelocity_.addDependency ( wMcamera_ );
   
}

SwayMotionCorrection::~SwayMotionCorrection ()
{
  task_.kill ();
}

void
SwayMotionCorrection::initialize (const vpHomogeneousMatrix& cdMo, int t)
{
  if (initialized_)
    return;

  for (unsigned i = 0; i < 2; ++i){
    E_[i] = 0.;
    integralLbk_[i] = 0.;
	  for (unsigned j = 0; j < 320; j++) {
		  E[i][j]=0.;
	  }
  }

  Einc =0; 
  
  for (unsigned i = 0; i < 3; ++i)
    inputComVel[i] = 0;
   
  cdMo_ = cdMo;
  cdMc_ = cdMo_ * convert(cMo_ (t).inverse ());

  FT_.buildFrom(cdMc_);
  FThU_.buildFrom(cdMc_);
  task_.addFeature (FT_);
  task_.addFeature (FThU_);
  initialized_ = true;
}

bool
SwayMotionCorrection::shouldStop (vpHomogeneousMatrix Error,double ErrorMoy) const
{
  vpColVector error (4);
  // Stop when robot is between its feet
  error[0] = Error[1][3]-ErrorMoy;
  // And when the error without sway motion is low
  error[1] = Error[0][3];
  error[2] = Error[1][3];
  error[3] = task_.error[5];

  return error.infinityNorm() < minThreshold_;
}

void
SwayMotionCorrection::stop ()
{
  std::cerr << "stopping the control law" << std::endl;
  initialized_ = false;
}

// 1. Compute camera velocity (cVelocity) using the standard servoing
// techniques. See vpServo doc.
//
// 2. Take into account the sway motion by adding a correcting term to
// the camera velocity.
//

// 3. Change velocity frame.
//
// 4. Check whether we should stop.
ml::Vector&
SwayMotionCorrection::updateVelocity (ml::Vector& velWaist, int t)
{      // --------------------- debug ---------------------------- //
      ofstream fichier("/home/mgeisert/debug.txt", ios::out | ios::app);
      
  if (velWaist.size () != 3)
    {
      velWaist.resize (3);
      velWaist.setZero ();
    }
  if (!initialized_)
    {
      velWaist.setZero ();
      return velWaist;
    }
 
  cdMc_ = cdMo_ * convert(cMo_ (t).inverse ());
  
  // Compute distance between camera and desired camera, in the waist frame
  vpHomogeneousMatrix waistTc =
    convert (wMwaist_ (t).inverse () * wMcamera_ (t));
  vpHomogeneousMatrix cMcd = cdMc_.inverse();
    for (int i=0 ; i<3 ; i++ ) {
      waistTc[i][3]=0;
  }
  
  vpHomogeneousMatrix cwaistMcd;
  cwaistMcd = waistTc * cMcd;
  
  // Compute sway motion correction   
   vpColVector ComVel (6);
  for (unsigned i = 0; i < 2; ++i) 
    ComVel[i] = inputdcom_ (t)(i);  
  ComVel[2] = ComVel[3] = ComVel[4] = ComVel[5] = 0;
  vpVelocityTwistMatrix waistVw (convert (wMwaist_ (t)).inverse());
  ComVel = waistVw * ComVel ;
  
  vpColVector bk (2);
  bk[0] = ComVel[0] - inputComVel[0];
  bk[1] = ComVel[1] ;//- inputComVel[1]; Pb lateral velocity
  
  integralLbk_ += bk * STEP;
  E_ += integralLbk_ * STEP;
    
  for (int i=0; i < 2; i++) {
	 E_tT[i] = (E_[i] - E[i][Einc])/T;
	}
  for (int i=0; i < 2; i++) {
	 E[i][Einc]=E_[i]; // save E_ to calculate E_tT at the next period
    }
  Einc++;
  if (Einc == 320) { Einc = 0; } 
  
  // Add sway motion correction to the distance between cam and desired cam
  cwaistMcd [0][3] = cwaistMcd[0][3] +  (integralLbk_[0]- E_tT[0]);
  cwaistMcd [1][3] = cwaistMcd[1][3] +  (integralLbk_[1]- E_tT[1]);
    
  // Compute new control law.
  FT_.buildFrom (cdMc_);
  FThU_.buildFrom (cdMc_);
  vpColVector cVelocity_ = task_.computeControlLaw ();
  
  // recompute control law (Visp-servoing overtaking)
  cVelocity_[0] = 1.5*lambda_ * cwaistMcd[0][3];
  cVelocity_[1] = 2*lambda_ * cwaistMcd[1][3];
      
  // Compute bounded velocity.
  vpColVector velWaistVispBounded =
    this->velocitySaturation (cVelocity_);

  // Modify the lateral velocity ( dead zone, velWaistVispBounded[1] must be >= 0.03 )
  if (velWaistVispBounded[1] > 0 && velWaistVispBounded[1] < 0.05 && velWaistVispBounded[1] > 0.01) { velWaistVispBounded[1] = 0.05; }
  if (velWaistVispBounded[1] < 0 && velWaistVispBounded[1] > -0.05 && velWaistVispBounded[1] < -0.010) { velWaistVispBounded[1] = -0.05; }

  // Fill signal.
  for (unsigned i = 0; i < 3; ++i)
    velWaist (i) = velWaistVispBounded[i]; 
     fichier << cwaistMcd[0][3] << " " << cwaistMcd[1][3] << " " << wMcamera_(t)(0,3) << " " << wMcamera_(t)(1,3) << " " << task_.error[0] << " " << task_.error[1] << " " << (waistTc * cMcd)[0][3] << " " << (waistTc * cMcd)[1][3] << " " << integralLbk_[1] << " " << E_tT[1] << " " << integralLbk_[1]- E_tT[1] << " " << task_.error[5] << " " << ComVel[1] << " " << ComVel[0]<< " " << velWaistVispBounded[0] << " " << velWaistVispBounded[1] << " " << velWaistVispBounded[2] << endl;

  // If the error is low, stop.
  if (shouldStop(waistTc * cMcd,cwaistMcd[1][3])){
     stop (); }
    
  // Save Velocity command for the next loop  
  for (unsigned i = 0; i < 3; ++i)
  inputComVel[i] = velWaist(i);
  
  return velWaist;
}


vpColVector
SwayMotionCorrection::velocitySaturation (const vpColVector& velocity)
{
  // compute the 3ddl vector corresponding to the input
  vpColVector RawVel3ddl (3);
  RawVel3ddl[0] = velocity[0];
  RawVel3ddl[1] = velocity[1];
  RawVel3ddl[2] = velocity[5];


  //Saturation of the velocity
  for ( int i = 0 ; i < 3 ; i++ ) {
	  if ( RawVel3ddl[i] > vmax_[i] )
	    RawVel3ddl[i] = vmax_[i];
	  if ( RawVel3ddl[i] < -vmax_[i] )
	    RawVel3ddl[i] = -vmax_[i];
	  }
  //Lower saturation for the backward walking
  if ( RawVel3ddl[0] < -vmax_[0]/2 )
    RawVel3ddl[0] = -vmax_[0]/2;
	  
  vpColVector result(3);
  for (int i=0; i<3;++i) 
    result[i] = RawVel3ddl[i];
  return result;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN
(SwayMotionCorrection, "SwayMotionCorrection");


namespace command
{
  namespace swayMotionCorrection
  {
    Initialize::Initialize (SwayMotionCorrection& entity,
			    const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::MATRIX) (Value::INT),
	 docstring)
    {}

    Value
    Initialize::doExecute ()
    {
      SwayMotionCorrection& entity =
	static_cast<SwayMotionCorrection&> (owner ());

      std::vector<Value> values = getParameterValues ();
      ml::Matrix M = values[0].value ();
      int t = values[1].value ();

      vpHomogeneousMatrix cdMo = convert (M);

      entity.initialize (cdMo, t);
      return Value ();
    }

    SetMaximumVelocity::SetMaximumVelocity (SwayMotionCorrection& entity,
					    const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::DOUBLE) (Value::DOUBLE) (Value::DOUBLE),
	 docstring)
    {}

    Value
    SetMaximumVelocity::doExecute ()
    {
      SwayMotionCorrection& entity =
	static_cast<SwayMotionCorrection&> (owner ());

      std::vector<Value> values = getParameterValues ();
      double dx = values[0].value ();
      double dy = values[1].value ();
      double dtheta = values[2].value ();

      entity.setMaximumVelocity(dx, dy, dtheta);
      return Value ();
    }

  } // end of namespace swayMotionCorrection.
} // end of namespace command.
