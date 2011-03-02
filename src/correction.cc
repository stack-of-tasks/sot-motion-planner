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
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <dynamic-graph/command.h>

#include "common.hh"

namespace ml = maal::boost;
namespace dg = dynamicgraph;
namespace sot = dg::sot;

namespace ublas = boost::numeric::ublas;

namespace
{
  class E
  {
  public:
    explicit E(const double& tmin,
	       const double& tmax,
	       const double& finalValue);

    double operator () (const double& t) const;

    const double tmin_;
    const double tmax_;
    const double finalValue_;

  private:
    double a_;
    double b_;
  };

  E::E (const double& tmin, const double& tmax, const double& finalValue)
    : tmin_ (tmin),
      tmax_ (tmax),
      finalValue_ (finalValue),
      a_ (),
      b_ ()
  {
    assert (tmin <= tmax);

    const double T = tmax_ - tmin_;
    const double T_2 = T * T;
    const double T_3 = T_2 * T;

    a_ = -2. / T_3 * finalValue;
    b_ = 3. * finalValue_ / T_2;
  }

  double
  E::operator () (const double& t) const
  {
    assert (t >= tmin_);
    assert (t <= tmax_);

    const double t_ = t - tmin_;
    const double t_2 = t_ * t_;
    const double t_3 = t_2 * t_;
    return a_ * t_3 + b_ * t_2 + 0. * t + 0.;
  }

} // end of anonymous namespace.


class Correction : public dg::Entity
{
public:
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalInMatrixHomo_t;
  typedef dg::SignalPtr<ml::Matrix, int> signalInMatrix_t;
  typedef dg::SignalPtr<ml::Vector, int> signalInVector_t;

  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int>
  signalOutMatrixHomo_t;
  typedef dg::SignalTimeDependent<ml::Matrix, int> signalOutMatrix_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalOutVector_t;

  static const std::string CLASS_NAME;

  explicit Correction (const std::string& name);

  sot::MatrixHomogeneous&
  computeLeftFootCorrectedTrajectory (sot::MatrixHomogeneous& res, int t);
  sot::MatrixHomogeneous&
  computeRightFootCorrectedTrajectory (sot::MatrixHomogeneous& res, int t);
  ml::Vector&
  computeComCorrectedTrajectory (ml::Vector& res, int t);

  ml::Vector getCurrentLeftFootCorrection (int t);
  ml::Vector getCurrentRightFootCorrection (int t);
  ml::Vector getCurrentComCorrection (int t);

private:
  struct OneAxisCorrection
  {
    explicit OneAxisCorrection ()
      : initialOffset_ (0.),
	correction_ ()
    {
    }

    double operator () (const double& t)
    {
      if (!correction_)
	return initialOffset_;

      if (t > correction_->tmax_)
	correction_.reset ();

      return initialOffset_ + (*correction_) (t);
    }

    void updateCorrection (const double& t,
			   const double& stepLength,
			   const double& offset)
    {
      if (correction_)
	return;
      correction_ = boost::make_shared<E> (t, t + stepLength, offset);
    }

    double initialOffset_;
    boost::shared_ptr<E> correction_;
  };

  /// \brief How many time does a step length?
  double stepLength_;

  /// \name Planned trajectory.
  /// \{
  signalInMatrixHomo_t trajectoryLeftFootIn_;
  signalInMatrixHomo_t trajectoryRightFootIn_;
  signalInVector_t trajectoryComIn_;
  /// \}

  /// \brief Current offset between planned free flyer and real free
  /// flyer.
  signalInVector_t offsetIn_;

  /// \name Corrected trajectory.
  /// \{
  signalOutMatrixHomo_t trajectoryLeftFootOut_;
  signalOutMatrixHomo_t trajectoryRightFootOut_;
  signalOutVector_t trajectoryComOut_;
  /// \}

  /// \name Current correction
  /// \{
  std::vector<OneAxisCorrection> leftFootCurrentCorrection_;
  std::vector<OneAxisCorrection> rightFootCurrentCorrection_;
  std::vector<OneAxisCorrection> comCurrentCorrection_;
  /// \}
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Correction, "Correction");

Correction::Correction (const std::string& name)
  : Entity (name),

    trajectoryLeftFootIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "trajectoryLeftFoot")),
    trajectoryRightFootIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "trajectoryRightFoot")),
    trajectoryComIn_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "trajectoryCom")),


    offsetIn_
    (dg::nullptr, MAKE_SIGNAL_STRING (name, true, "Vector", "offset")),


    trajectoryLeftFootOut_
    (boost::bind
     (&Correction::computeLeftFootCorrectedTrajectory, this, _1, _2),
     trajectoryLeftFootIn_ << offsetIn_,
     MAKE_SIGNAL_STRING (name, false, "MatrixHomo", "trajectoryLeftFoot")),

    trajectoryRightFootOut_
    (boost::bind
     (&Correction::computeRightFootCorrectedTrajectory, this, _1, _2),
     trajectoryRightFootIn_ << offsetIn_,
     MAKE_SIGNAL_STRING (name, false, "MatrixHomo", "trajectoryRightFoot")),

    trajectoryComOut_
    (boost::bind
     (&Correction::computeComCorrectedTrajectory, this, _1, _2),
     trajectoryComIn_ << offsetIn_,
     MAKE_SIGNAL_STRING (name, false, "Vector", "trajectoryCom")),

    leftFootCurrentCorrection_ (3),
    rightFootCurrentCorrection_ (3),
    comCurrentCorrection_ (2)
{
  signalRegistration
    (trajectoryLeftFootIn_ << trajectoryRightFootIn_ << trajectoryComIn_
     << offsetIn_
     << trajectoryLeftFootOut_ << trajectoryRightFootOut_ << trajectoryComOut_);

}


sot::MatrixHomogeneous&
Correction::computeLeftFootCorrectedTrajectory (sot::MatrixHomogeneous& res,
						int t)
{
  ml::Vector leftFootCorrection = getCurrentLeftFootCorrection (t);
  // Update X
  res (0, 3) += leftFootCorrection (0);
  // Update Y
  res (1, 3) += leftFootCorrection (1);

  // FIXME: rotate to update theta.
  return res;
}

sot::MatrixHomogeneous&
Correction::computeRightFootCorrectedTrajectory (sot::MatrixHomogeneous& res,
						 int t)
{
  ml::Vector rightFootCorrection = getCurrentRightFootCorrection (t);
  // Update X
  res (0, 3) += rightFootCorrection (0);
  // Update Y
  res (1, 3) += rightFootCorrection (1);

  // FIXME: rotate to update theta.
  return res;
}

ml::Vector&
Correction::computeComCorrectedTrajectory (ml::Vector& res, int t)
{
  ml::Vector comCorrection = getCurrentComCorrection (t);
  // Update X
  res (0) += comCorrection (0);
  // Update Y
  res (1) += comCorrection (1);

  return res;
}

ml::Vector
Correction::getCurrentLeftFootCorrection (int t)
{
  ml::Vector res (3);
  for (unsigned i = 0; i < 3; ++i)
    {
      //FIXME: the offset is wrong, missing ff offset -> feet offset
      //conversion.
      leftFootCurrentCorrection_[i].updateCorrection
	(t, stepLength_, offsetIn_ (t) (i));
      res (0) = leftFootCurrentCorrection_[i] (t);
    }
  return res;
}

ml::Vector
Correction::getCurrentRightFootCorrection (int t)
{
  ml::Vector res (3);
  for (unsigned i = 0; i < 3; ++i)
    {
      //FIXME: the offset is wrong, missing ff offset -> feet offset
      //conversion.
      rightFootCurrentCorrection_[i].updateCorrection
	(t, stepLength_, offsetIn_ (t) (i));
      res (0) = rightFootCurrentCorrection_[i] (t);
    }
  return res;
}

ml::Vector
Correction::getCurrentComCorrection (int t)
{
  ml::Vector res (2);
  for (unsigned i = 0; i < 2; ++i)
    {
      //FIXME: the offset is wrong, missing ff offset -> com offset
      //conversion.
      comCurrentCorrection_[i].updateCorrection
	(t, stepLength_, offsetIn_ (t) (i));
      res (0) = comCurrentCorrection_[i] (t);
    }
  return res;
}
