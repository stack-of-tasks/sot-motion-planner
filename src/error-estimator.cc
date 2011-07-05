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
    wMsensor_ (),
    position_ (dg::nullptr,
	       MAKE_SIGNAL_STRING (name, true, "Vector", "position")),
    positionTimestamp_
    (dg::nullptr,
     MAKE_SIGNAL_STRING (name, true, "Vector", "positionTimestamp")),
    planned_ (dg::nullptr,
	    MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "planned")),
    error_ (INIT_SIGNAL_OUT ("error", ErrorEstimator::updateError, "Vector")),
    dbgPositionWorldFrame_ (
			    INIT_SIGNAL_OUT
			    ("dbgPositionWorldFrame",
			     ErrorEstimator::updateDbgPositionWorldFrame,
			     "MatrixHomo")),

    dbgPlanned_ (
		 INIT_SIGNAL_OUT
		 ("dbgPlanned",
		  ErrorEstimator::updateDbgPlanned,
		  "MatrixHomo")),
    dbgIndex_ (
	       INIT_SIGNAL_OUT
	       ("dbgIndex",
		ErrorEstimator::updateDbgIndex,
		"Vector")),

    dbgPositionWorldFrameValue_ (),
    dbgPlannedValue_ (),
    dbgIndexValue_ (2),

    referenceTrajectory_ (dg::nullptr),
    plannedPositions_ (),
    started_ (false)
{
  signalRegistration (position_ << positionTimestamp_ << planned_ << error_
		      << dbgPositionWorldFrame_
		      << dbgPlanned_
		      << dbgIndex_);
  error_.setNeedUpdateFromAllChildren (true);

  dbgPositionWorldFrame_.setNeedUpdateFromAllChildren (true);
  dbgPlanned_.setNeedUpdateFromAllChildren (true);
  dbgIndex_.setNeedUpdateFromAllChildren (true);

  std::string docstring;
  addCommand ("setReferenceTrajectory",
	      new command::SetReferenceTrajectory<ErrorEstimator>
	      (*this, docstring));

  addCommand ("setSensorToWorldTransformation",
	      new dg::command::Setter<ErrorEstimator, ml::Matrix>
	      (*this, &ErrorEstimator::sensorToWorldTransformation, docstring));
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

  for (unsigned i = 0; i < plannedPositions_.size (); ++i)
    {
      const pair_t& e = plannedPositions_[plannedPositions_.size () - 1 - i];
      if (boost::get<0> (e) < time)
	return plannedPositions_.size () - 1 - i;
    }
  return 0;
}

ml::Vector&
ErrorEstimator::updateError (ml::Vector& res, int t)
{
  using namespace boost::gregorian;
  using namespace boost::posix_time;

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
      plannedPositions_.reserve (5 * *size);
      started_ = true;
    }

  //FIXME: here we suppose implicit sync between feet follower and
  //error estimation.
  static const int delta_usec = 1500 * 1000; // 150ms.
  plannedPositions_.push_back
    (boost::make_tuple
     (boost::posix_time::microsec_clock::universal_time ()
      + microseconds (delta_usec),
      t, planned_ (t)));

  if (positionTimestamp_ (t).size () != 2)
    return res;
  size_t index = timestampToIndex (positionTimestamp_ (t));

  if (index >= plannedPositions_.size ())
    return res;

  sot::MatrixHomogeneous planned = boost::get<2> (plannedPositions_[index]);
  sot::MatrixHomogeneous estimated = wMsensor_ *
    XYThetaToMatrixHomogeneous (position_ (t));

  sot::MatrixHomogeneous error = estimated.inverse () * planned;

  // Express the error in the world frame instead
  // of the leftAnkle frame.
  sot::MatrixHomogeneous errorW =
    planned_ (t) * error * planned_ (t).inverse ();

  ml::Vector error_xytheta = MatrixHomogeneousToXYTheta (errorW);

  if (maxError_)
    for (unsigned i = 0; i < 3; ++i)
      {
	if (error_xytheta (i) > (*maxError_)[i])
	  error_xytheta (i) = (*maxError_)[i];
	if (error_xytheta (i) < -(*maxError_)[i])
	  error_xytheta (i) = -(*maxError_)[i];
      }
  res = error_xytheta;

  // Update debugging information.
  dbgPositionWorldFrameValue_ = estimated;
  dbgPlannedValue_ = planned;

  dbgIndexValue_ (0) = index;
  dbgIndexValue_ (1) = plannedPositions_.size ();

  return res;
}

sot::MatrixHomogeneous&
ErrorEstimator::updateDbgPositionWorldFrame (sot::MatrixHomogeneous& res, int t)
{
  res = dbgPositionWorldFrameValue_;
  return res;
}

sot::MatrixHomogeneous&
ErrorEstimator::updateDbgPlanned (sot::MatrixHomogeneous& res, int t)
{
  res = dbgPlannedValue_;
  return res;
}

ml::Vector&
ErrorEstimator::updateDbgIndex (ml::Vector& res, int t)
{
  res = dbgIndexValue_;
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
