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

static const double STEP = 0.005;

vpHomogeneousMatrix
convert(const sot::MatrixHomogeneous& src)
{
  vpHomogeneousMatrix dst;
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      dst[i][j] = src (i, j);
  return dst;
}

vpHomogeneousMatrix
convert(const ml::Matrix& src)
{
  vpHomogeneousMatrix dst;
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      dst[i][j] = src (i, j);
  return dst;
}

ml::Vector
convert(const vpColVector& src)
{
  ml::Vector dst;
  dst.resize (src.getRows ());
  for (unsigned i = 0; i < dst.size (); ++i)
    dst (i) = src[i];
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
  vpVelocityTwistMatrix fromComToCameraTwist (int t);

  /// \brief Make sure that the velocity stays lower than vmax.
  vpColVector velocitySaturation (const vpColVector& velocity);

  /// \brief Update PG velocity callback.
  ml::Vector& updateVelocity (ml::Vector& v, int);


  ml::Vector& updateDbgCorrectedE (ml::Vector& v, int)
  {
    v = convert (correctedE_);
    return v;
  }
  ml::Vector& updateDbgE (ml::Vector& v, int)
  {
    v = convert (task_.error);
    return v;
  }

  ml::Vector&
  updateDbgVelocityWithoutCorrection (ml::Vector& v, int)
  {
    v = convert (velocityWithoutCorrection_);
    return v;
  }

  ml::Vector&
  updateDbgcMoWithCorrection (ml::Vector& v, int t)
  {
    vpHomogeneousMatrix cMo = convert(cMo_(t));
    vpPoseVector posecMo(cMo);
    //FIXEME: instead of modifying directly the pose vector,
    // transform the correction into an homogeneous matrix ...
    vpColVector poseccMo = posecMo - integralLbk_ + E_;
    v = convert(poseccMo);
    return v;
  }

  /// \brief Is the error lower enough to stop?
  bool shouldStop(vpColVector& error) const;

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

  /// \brief Input pattern generator velocity (signal).
  signalVectorIn_t inputPgVelocity_;
  /// \brief Output pattern generator velocity (signal).
  signalVectorOut_t outputPgVelocity_;

  /// \brief c*Mc
  signalMatrixHomoIn_t cMo_;
  signalVectorIn_t cMoTimestamp_;

  /// \brief waist position w.r.t world frame.
  signalMatrixHomoIn_t wMwaist_;
  /// \brief com position w.r.t world frame.
  signalVectorIn_t wMcom_;
  /// \brief Camera position w.r.t. world frame.
  signalMatrixHomoIn_t wMcamera_;

  /// \brief Center of mass jacobian.
  signalMatrixIn_t Jcom_;
  /// \brief Waist jacobian.
  signalMatrixIn_t Jwaist_;
  /// \brief Joint velocities \dot{\mathbf{qdot}}
  signalVectorIn_t qdot_;

  signalVectorOut_t dbgCorrectedE_;
  signalVectorOut_t dbgE_;
  signalVectorOut_t dbgVelocityWithoutCorrection_;
  signalVectorOut_t dbgcMoWithCorrection_;

  /// \brief If error is lower than this threshold then stop.
  double minThreshold_;

  /// \brief Error accumulation.
  vpColVector E_;
  vpColVector correctedE_;

  vpColVector velocityWithoutCorrection_;
  vpColVector cMoWithCorrection_;

  /// \brief FIXME
  vpColVector integralLbk_;
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
    lambda_ (0.1), //FIXME:
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
    cMoTimestamp_ (dg::nullptr,
		   MAKE_SIGNAL_STRING
		   (name, true, "Vector", "cMoTimestamp")),

    wMcom_ (dg::nullptr,
	      MAKE_SIGNAL_STRING
	      (name, true, "Vector", "wMcom")),
    wMwaist_ (dg::nullptr,
	      MAKE_SIGNAL_STRING
	      (name, true, "MatrixHomo", "wMwaist")),
    wMcamera_ (dg::nullptr,
	       MAKE_SIGNAL_STRING
	       (name, true, "MatrixHomo", "wMcamera")),

    Jcom_ (dg::nullptr,
	   MAKE_SIGNAL_STRING
	   (name, true, "Matrix", "Jcom")),
    Jwaist_ (dg::nullptr,
	     MAKE_SIGNAL_STRING
	     (name, true, "Matrix", "Jwaist")),
    qdot_ (dg::nullptr,
	   MAKE_SIGNAL_STRING
	   (name, true, "Vector", "qdot")),


    dbgCorrectedE_
    (INIT_SIGNAL_OUT
     ("dbgCorrectedE",
      SwayMotionCorrection::updateDbgCorrectedE, "Vector")),
    dbgE_
    (INIT_SIGNAL_OUT
     ("dbgE",
      SwayMotionCorrection::updateDbgE, "Vector")),
    dbgVelocityWithoutCorrection_
    (INIT_SIGNAL_OUT
     ("dbgVelocityWithoutCorrection",
      SwayMotionCorrection::updateDbgVelocityWithoutCorrection, "Vector")),
    dbgcMoWithCorrection_
    (INIT_SIGNAL_OUT
     ("dbgcMoWithCorrection",
      SwayMotionCorrection::updateDbgcMoWithCorrection, "Vector")),

    minThreshold_ (0.1),
    E_ (6),
    correctedE_ (6),
    velocityWithoutCorrection_ (6),
    cMoWithCorrection_ (),
    integralLbk_ (6)
{
  signalRegistration (inputPgVelocity_ << outputPgVelocity_
		      << cMo_ << cMoTimestamp_
		      << wMcom_ << wMwaist_ << wMcamera_
		      << Jcom_ << Jwaist_ << qdot_
		      << dbgCorrectedE_ << dbgE_
		      << dbgVelocityWithoutCorrection_
		      << dbgcMoWithCorrection_);

  for (unsigned i = 0; i < vmax_.getCols (); ++i)
    vmax_[i] = 0.;

  task_.setServo (vpServo::EYEINHAND_CAMERA);
  task_.setInteractionMatrixType (vpServo::CURRENT);
  task_.setLambda (lambda_);

  //outputPgVelocity_.setNeedUpdateFromAllChildren (true);

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
{
  task_.kill ();
}

void
SwayMotionCorrection::initialize (const vpHomogeneousMatrix& cdMo, int t)
{
  if (initialized_)
    return;

  for (unsigned i = 0; i < 6; ++i)
    E_[i] = 0.;

  cdMo_ = cdMo;
  cdMc_ = cdMo_ * convert(cMo_ (t).inverse ());

  FT_.buildFrom(cdMc_);
  FThU_.buildFrom(cdMc_);
  task_.addFeature (FT_);
  task_.addFeature (FThU_);
  initialized_ = true;
}

bool
SwayMotionCorrection::shouldStop (vpColVector& error) const
{
  // vpColVector e (3);
  // e[0] = error[0];
  // e[1] = error[2];
  // e[2] = error[4];

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
SwayMotionCorrection::updateVelocity (ml::Vector& velCom, int t)
{
  if (velCom.size () != 3)
    {
      velCom.resize (3);
      velCom.setZero ();
    }
  if (!initialized_ || inputPgVelocity_ (t).size() != 3)
    {
      velCom.resize (3);
      velCom.setZero ();
      return velCom;
    }
  if (Jcom_(t).nbRows() != 3 || Jcom_(t).nbCols() != qdot_(t).size())
    {
      std::cerr << "bad size" << std::endl;

      std::cout << Jcom_(t).nbCols() << std::endl;
      std::cout << Jcom_(t).nbRows() << std::endl;

      std::cout << qdot_(t).size() << std::endl;

      velCom.setZero ();
      return velCom;
    }

  // Compute the new desired camera position w.r.t. the current one.
  cdMc_ = cdMo_ * convert(cMo_ (t).inverse ());

  // Compute new control law.
  FT_.buildFrom (cdMc_);
  FThU_.buildFrom (cdMc_);

  // Compute com velocity.
  // We need both translation and rotation so the yaw rotation
  // is also retrieved from the waist.
  //
  // We make the assumption that the com frame and the waist frame are
  // aligned.
  vpColVector dcom (3);
  ml::Vector dcom_ = Jcom_(t) * qdot_(t);
  ml::Vector dwaist_ = Jwaist_(t) * qdot_(t);
  for (unsigned i = 0; i < 2; ++i)
    dcom[i] = dcom_(i);
  dcom[2] = dwaist_(5);

  // Convert input com velocity.
  vpColVector inputComVel (3);
  for (unsigned i = 0; i < 3; ++i)
    {
      inputComVel[i] = inputPgVelocity_ (t) (i);
    }

  // Compute the difference between the reference and the current
  // velocity.
  vpColVector bk (6);
  bk[0] = inputComVel[0] - dcom[0];
  bk[1] = inputComVel[1] - dcom[1];
  bk[2] = bk[3] = bk[4] = 0.;
  bk[5] = inputComVel[2] - dcom[2];


  // Here we compute the control law without the correction
  // but we only use the interaction matrix L.
  // The result can be used as a reference w/o correction.
  velocityWithoutCorrection_ = task_.computeControlLaw ();

  // Change the velocity frame from camera to com.
  vpVelocityTwistMatrix camVcom = fromComToCameraTwist (t);

  vpColVector swayMotionCorrection = task_.L * camVcom * bk;

  //FIXME: should we change this to t(cmo-1) - t(cmo) ?
  integralLbk_ += swayMotionCorrection * STEP;
  E_ += integralLbk_ * STEP;

  vpColVector correctedError = task_.error - integralLbk_ + E_;
  double lambda = task_.lambda.getLastValue ();
  vpColVector correctedVcam = -1. * lambda * task_.L.pseudoInverse () * correctedError;

  vpColVector correctedVelCom = camVcom.inverse () * correctedVcam;

  // Compute bounded camera velocity.
  vpColVector correctedVelComBounded =
    this->velocitySaturation (correctedVelCom);

  // If the error is low, stop.
  if (shouldStop(correctedError))
    {
      stop ();
      for (unsigned i = 0; i < 3; ++i)
	velCom (i) = 0.;
    }
  else
    // Fill signal.
    for (unsigned i = 0; i < 3; ++i)
      velCom (i) = correctedVelComBounded[i];
  return velCom;
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
SwayMotionCorrection::fromComToCameraTwist (int t)
{
  ml::Vector wMcom (4);
  for (unsigned i = 0; i < 3; ++i)
    wMcom (i) = wMcom_ (t) (i);
  wMcom (3) = 1.;

  ml::Vector cameraMcom =
    wMcamera_ (t).inverse () * wMcom;
  vpHomogeneousMatrix cameraMcom_;
  for (unsigned i = 0; i < 3; ++i)
    cameraMcom_[i][3] = cameraMcom (i);
  vpVelocityTwistMatrix cameraVcom (cameraMcom_);
  return cameraVcom;
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
