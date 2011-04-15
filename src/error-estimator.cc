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
  ptime_t time (date(1970,1,1), seconds (sec) + microseconds (usec));

  typedef boost::numeric::converter<size_t, int64_t> Int64_t2Size_t;

  typedef std::pair<ptime_t, sot::MatrixHomogeneous> pair_t;

  BOOST_FOREACH (const pair_t& e, waistPositions_)
    {
      if (e.first >= time)
	{
	  int64_t idx =
	    time_duration::ticks_per_second () /
	    ((e.first - time).ticks () * STEPS_PER_SECOND);
	  return Int64_t2Size_t::convert (idx);
	}
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
      waistPositions_.reserve (*size);
      started_ = true;
    }

  //FIXME: here we suppose implicit sync between feet follower and
  //error estimation.
  waistPositions_.push_back
    (std::make_pair (boost::posix_time::microsec_clock::universal_time (),
		     waist_ (t)));

  if (positionTimestamp_ (t).size () != 2)
    return res;
  size_t index = timestampToIndex (positionTimestamp_ (t));

  if (index >= waistPositions_.size ())
    return res;

  sot::MatrixHomogeneous planned = waistPositions_[index].second;
  sot::MatrixHomogeneous estimated = worldTransformation_ *
    XYThetaToMatrixHomogeneous (position_ (t));

  sot::MatrixHomogeneous error = planned * estimated.inverse ();
  res = MatrixHomogeneousToXYTheta (error);
  return res;
}

void
ErrorEstimator::setReferenceTrajectory (FeetFollower* ptr)
{
  referenceTrajectory_ = ptr;
  started_ = false;
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (ErrorEstimator, "ErrorEstimator");
