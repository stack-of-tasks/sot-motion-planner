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

#include <boost/numeric/conversion/converter.hpp>
#include <boost/date_time/date.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

#include <dynamic-graph/command-setter.h>

#include "common.hh"
#include "error-estimator.hh"
#include "set-reference-trajectory.hh"

static const int STEPS_PER_SECOND = 200;

namespace command
{
  namespace errorEstimator
  {
    SetSafetyLimits::SetSafetyLimits
    (ErrorEstimator& entity, const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::DOUBLE) (Value::DOUBLE) (Value::DOUBLE),
	 docstring)
    {}

    Value SetSafetyLimits::doExecute ()
    {
      ErrorEstimator& entity =
	static_cast<ErrorEstimator&> (owner ());

      std::vector<Value> values = getParameterValues ();
      double maxErrorX = values[0].value ();
      double maxErrorY = values[1].value ();
      double maxErrorTheta = values[2].value ();

      entity.setSafetyLimits (maxErrorX, maxErrorY, maxErrorTheta);
      return Value ();
    }

    UnsetSafetyLimits::UnsetSafetyLimits
    (ErrorEstimator& entity, const std::string& docstring)
      : Command
	(entity,
	 std::vector<Value::Type> (),
	 docstring)
    {}

    Value
    UnsetSafetyLimits::doExecute ()
    {
      ErrorEstimator& entity =
	static_cast<ErrorEstimator&> (owner ());
      entity.unsetSafetyLimits ();
      return Value ();
    }
  } // end of errorEstimator.
} // end of namespace command.


ErrorEstimator::ErrorEstimator (const std::string& name)
  : dg::Entity (name),
    worldTransformation_ (),
    position_ (dg::nullptr,
	       MAKE_SIGNAL_STRING (name, true, "Vector", "position")),
    positionTimestamp_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "positionTimestamp")),
    waist_ (dg::nullptr,
	    MAKE_SIGNAL_STRING (name, true, "Vector", "waist")),
    error_ (INIT_SIGNAL_OUT ("error", ErrorEstimator::updateError, "Vector")),
    referenceTrajectory_ (dg::nullptr),
    waistPositions_ (),
    started_ (false)
{
  signalRegistration (position_ << positionTimestamp_ << waist_ << error_);
  error_.setNeedUpdateFromAllChildren (true);

  std::string docstring;
  addCommand ("setReferenceTrajectory",
	      new command::SetReferenceTrajectory<ErrorEstimator>
	      (*this, docstring));

  addCommand ("setWorldTransformation",
	      new dg::command::Setter<ErrorEstimator, ml::Matrix>
	      (*this, &ErrorEstimator::worldTransformation, docstring));

  addCommand ("setSafetyLimits",
	      new command::errorEstimator::SetSafetyLimits
	      (*this, docstring));
  addCommand ("unsetSafetyLimits",
	      new command::errorEstimator::UnsetSafetyLimits
	      (*this, docstring));
}

ErrorEstimator::~ErrorEstimator ()
{}

// FIXME: don't use the current one but synchronize to take
// into account the delay in packet transmission.
size_t
ErrorEstimator::timestampToIndex (const ml::Vector& timestamp)
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

  typedef boost::numeric::converter<long int, double> Double2Long;

  long int sec = Double2Long::convert (timestamp (0));
  long int usec = Double2Long::convert (timestamp (1));

  //FIXME: fail at midnight.
  ptime_t time (day_clock::universal_day (),
		seconds (sec) + microseconds (usec));

  typedef boost::numeric::converter<size_t, int64_t> Int64_t2Size_t;

  typedef boost::tuple<ptime_t, int, sot::MatrixHomogeneous> pair_t;

  for (unsigned i = 0; i < waistPositions_.size (); ++i)
    {
      const pair_t& e = waistPositions_[waistPositions_.size () - 1 - i];
      if (boost::get<0> (e) < time)
	return waistPositions_.size () - 1 - i;
    }
  return 0;
}

ml::Vector&
ErrorEstimator::updateError (ml::Vector& res, int t)
{
  if (res.size () != 3)
    res.resize (3);
  res.setZero ();

  if (!referenceTrajectory_)
    return res;

  referenceTrajectory_->update (t);

  if (referenceTrajectory_->started () && !started_)
    {
      boost::optional<int> size = referenceTrajectory_->trajectorySize ();
      assert (size);
      waistPositions_.reserve (5 * *size);
      started_ = true;
    }

  //FIXME: here we suppose implicit sync between feet follower and
  //error estimation.
  waistPositions_.push_back
    (boost::make_tuple
     (boost::posix_time::microsec_clock::universal_time (),
      t, waist_ (t)));

  if (positionTimestamp_ (t).size () != 2)
    return res;
  size_t index = timestampToIndex (positionTimestamp_ (t));

  if (index >= waistPositions_.size ())
    return res;

  sot::MatrixHomogeneous planned = boost::get<2> (waistPositions_[index]);
  sot::MatrixHomogeneous estimated = worldTransformation_ *
    XYThetaToMatrixHomogeneous (position_ (t));

  sot::MatrixHomogeneous error = planned * estimated.inverse ();
  ml::Vector error_xytheta = MatrixHomogeneousToXYTheta (error);

  if (maxError_)
    for (unsigned i = 0; i < 3; ++i)
      {
	if (error_xytheta (i) > (*maxError_)[i])
	  error_xytheta (i) = (*maxError_)[i];
	if (error_xytheta (i) < -(*maxError_)[i])
	  error_xytheta (i) = -(*maxError_)[i];
      }
  res = error_xytheta;
  return res;
}

void
ErrorEstimator::setReferenceTrajectory (FeetFollower* ptr)
{
  referenceTrajectory_ = ptr;
  started_ = false;
}

void
ErrorEstimator::setSafetyLimits (const double& maxErrorX,
				 const double& maxErrorY,
				 const double& maxErrorTheta)
{
  boost::array<double, 3> max;
  max[0] = maxErrorX;
  max[1] = maxErrorY;
  max[2] = maxErrorTheta;
  maxError_ = max;
}

void
ErrorEstimator::unsetSafetyLimits ()
{
  maxError_.reset ();
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (ErrorEstimator, "ErrorEstimator");
