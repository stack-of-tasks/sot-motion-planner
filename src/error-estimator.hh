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

#ifndef SOT_MOTION_PLANNER_ERROR_ESTIMATOR_HH
# define SOT_MOTION_PLANNER_ERROR_ESTIMATOR_HH
# include <string>
# include <utility>

# include <boost/array.hpp>
# include <boost/date_time/posix_time/posix_time_types.hpp>
# include <boost/shared_ptr.hpp>
# include <boost/tuple/tuple.hpp>

# include <jrl/mal/boost.hh>

# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/pool.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/signal-ptr.h>

# include <sot/core/matrix-homogeneous.hh>

# include "common.hh"
# include "discretized-trajectory.hh"
# include "feet-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

class FeetFollower;
class ErrorEstimator;

namespace command
{
  namespace errorEstimator
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class SetSafetyLimits : public Command
    {
    public:
      SetSafetyLimits (ErrorEstimator& entity,
		       const std::string& docstring);
      virtual Value doExecute ();
    };

    class UnsetSafetyLimits : public Command
    {
    public:
      UnsetSafetyLimits (ErrorEstimator& entity,
			 const std::string& docstring);
      virtual Value doExecute ();
    };
  } // end of namespace errorEstimator.
} // end of namespace command.


class ErrorEstimator : public dg::Entity
{
 public:
  typedef boost::posix_time::ptime ptime_t;

  static const std::string CLASS_NAME;

  typedef dg::SignalPtr<ml::Vector, int> signalVectorIn_t;
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalVectorOut_t;

  explicit ErrorEstimator (const std::string& name);
  virtual ~ErrorEstimator ();

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

  void setReferenceTrajectory (FeetFollower* ptr);

  ml::Vector& updateError (ml::Vector& res, int);

  void setSafetyLimits (const double& maxErrorX,
			const double& maxErrorY,
			const double& maxErrorTheta);
  void unsetSafetyLimits ();

protected:
  size_t timestampToIndex (const ml::Vector& timestamp);

  void worldTransformation (const ml::Matrix& wt)
  {
    worldTransformation_ = wt;
  }

  sot::MatrixHomogeneous worldTransformation_;

  /// \brief Robot position (X, Y, theta)
  signalVectorIn_t position_;
  /// \brief Robot position timestamp (time)
  signalVectorIn_t positionTimestamp_;

  /// \brief Current waist position.
  signalMatrixHomoIn_t waist_;
  /// \brief Planned robot position in the real robot frame.
  signalVectorOut_t error_;

  FeetFollower* referenceTrajectory_;

  std::vector<boost::tuple<ptime_t, unsigned, sot::MatrixHomogeneous> >
  waistPositions_;
  bool started_;

  boost::optional<boost::array<double, 3> > maxError_;
};

#endif //! SOT_MOTION_PLANNER_ERROR_ESTIMATOR_HH
