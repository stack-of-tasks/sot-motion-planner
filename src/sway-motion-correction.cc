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


#include "common.hh"

vpHomogeneousMatrix
convert(sot::MatrixHomogeneous src)
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

  void setMaximumVelocity(const double& dx, const double& dy, const double& dtheta)
  {
    vmax_[0] = dx;
    vmax_[1] = dy;
    vmax_[2] = dtheta;
  }

protected:
  vpVelocityTwistMatrix fromCameraToWaistTwist (int t);
  vpColVector velocitySaturation (const vpColVector& velocity);

  /// \brief Update PG velocity callback.
  ml::Vector& updateVelocity (ml::Vector& v, int);

  bool initialized_;

  double lambda_;
  vpColVector vmax_;

  /// \brief Set before starting computing control law.
  vpHomogeneousMatrix cdMo_;

  vpHomogeneousMatrix cdMc_;
  vpFeatureTranslation FT_;
  vpFeatureThetaU FThU_;

  vpServo task_;

  signalVectorIn_t inputPgVelocity_;
  signalVectorOut_t outputPgVelocity_;

  /// \brief c*Mc
  signalMatrixHomoIn_t cMo_;

  signalMatrixHomoIn_t wMwaist_;
  signalMatrixHomoIn_t wMcamera_;

  vpColVector cVelocity_;
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
    lambda_ (0.6), //FIXME:
    vmax_ (3),
    cdMc_ (),
    FT_ (vpFeatureTranslation::cdMc),
    FThU_ (vpFeatureThetaU::cdRc),

    task_ (),

    inputPgVelocity_ (dg::nullptr,
		      MAKE_SIGNAL_STRING (name, true, "Vector", "inputPgVelocity")),
    outputPgVelocity_ (INIT_SIGNAL_OUT
		       ("outputPgVelocity",
			SwayMotionCorrection::updateVelocity, "Vector")),
    cMo_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "cMo")),

    wMwaist_ (dg::nullptr,
	      MAKE_SIGNAL_STRING
	      (name, true, "MatrixHomo", "wMwaist")),
    wMcamera_ (dg::nullptr,
	       MAKE_SIGNAL_STRING
	       (name, true, "MatrixHomo", "wMcamera")),

    cVelocity_ (6)
{
  signalRegistration (inputPgVelocity_ << outputPgVelocity_ << cMo_
		      << wMwaist_ << wMcamera_);

  for (unsigned i = 0; i < vmax_.getCols (); ++i)
    vmax_[i] = 0.;

  task_.setServo (vpServo::EYEINHAND_CAMERA);
  task_.setInteractionMatrixType (vpServo::CURRENT);
  task_.setLambda (lambda_);

  std::string docstring;
  addCommand
    ("initialize",
     new command::swayMotionCorrection::Initialize
     (*this, docstring));

  addCommand
    ("setMaximumVelocity",
     new command::swayMotionCorrection::SetMaximumVelocity
     (*this, docstring));
}

SwayMotionCorrection::~SwayMotionCorrection ()
{}

void
SwayMotionCorrection::initialize (const vpHomogeneousMatrix& cdMo, int t)
{
  if (initialized_)
    return;

  cdMo_ = cdMo;
  cdMc_ = cdMo_ * convert(cMo_ (t).inverse ());

  FT_.buildFrom(cdMc_);
  FThU_.buildFrom(cdMc_);
  task_.addFeature (FT_);
  task_.addFeature (FThU_);
  initialized_ = true;
}

ml::Vector&
SwayMotionCorrection::updateVelocity (ml::Vector& velWaist, int t)
{
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

  FT_.buildFrom (cdMc_);
  FThU_.buildFrom (cdMc_);

  cVelocity_ = task_.computeControlLaw ();

  //vpMatrix L = task_.L;

  vpColVector velCam = this->velocitySaturation (cVelocity_);

  // Change the velocity frame from camera to waist
  vpVelocityTwistMatrix waistVcamera = fromCameraToWaistTwist (t);

  vpColVector velWaistVisp = waistVcamera * velCam;

  for (unsigned i = 0; i < 3; ++i)
    velWaist (i) = velWaistVisp[i];

  return velWaist;
}

vpColVector
SwayMotionCorrection::velocitySaturation (const vpColVector& velocity)
{
  vpColVector dv (0.05 * vmax_);
  vpColVector Vinf  = vmax_ - dv;
  vpColVector Vsup  = vmax_ + dv;

  // compute the 3ddl vector corresponding to the input
  vpColVector RawVel3ddl (3);
  RawVel3ddl[0] = velocity[0];
  RawVel3ddl[1] = velocity[1];
  RawVel3ddl[2] = velocity[5];

  // temporary abs value of the input velocity
  double absRawVel = 0.;

  // normalization factor shared for all components to keep vector
  // orientation.
  double fac = 1.;

  // for all the coeff
  for (int i = 0; i < 3; ++i)
    {
      absRawVel = std::fabs (RawVel3ddl[i]);

      fac = std::min (std::fabs(fac), vmax_[i] / (absRawVel + 0.00001));

      // to prevent from discontinuities
      if ((Vinf[i] <= absRawVel) && (absRawVel <= Vsup[i]))
	{
	  double newfac = 1 / (2 * dv[i] * absRawVel) *
	    ((absRawVel - Vinf[i]) * vmax_[i]
	     + (Vsup[i] - absRawVel) * Vinf[i]);

	  fac  = std::min (std::fabs(fac), std::fabs(newfac));
	}
    }

  vpColVector result(3);
  for (int i=0; i<3;++i)
    result[i] = RawVel3ddl[i] * fac;
  return result;
}

vpVelocityTwistMatrix
SwayMotionCorrection::fromCameraToWaistTwist (int t)
{
  vpHomogeneousMatrix waistMcamera =
    convert (wMwaist_ (t).inverse () * wMcamera_ (t));
  vpVelocityTwistMatrix waistVcamera (waistMcamera);
  return waistVcamera;
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
	(entity, std::vector<Value::Type>(), docstring)
    {}

    Value
    Initialize::doExecute ()
    {
      SwayMotionCorrection& entity =
	static_cast<SwayMotionCorrection&> (owner ());

      vpHomogeneousMatrix cdMo;
      int t;

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
