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

/// \brief Convert a 3d pose (4x4 homogeneous matrix) to 2d pose (x,y,yaw).
class ThreeToTwoDimensionPoseConverter : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  /// \brief Input vector signal.
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;
  /// \brief Output homogeneous matrix signal.
  typedef dg::SignalTimeDependent<ml::Vector, int>
  signalVectorOut_t;

  /// \name Constructor and destructor.
  /// \{
  explicit ThreeToTwoDimensionPoseConverter (const std::string& name);
  virtual ~ThreeToTwoDimensionPoseConverter ();
  /// \}

  /// \brief Signal callback.
  ml::Vector& update (ml::Vector& res, int);

protected:
  /// \brief Input signal.
  signalMatrixHomoIn_t in_;
  /// \brief Output signal.
  signalVectorOut_t out_;
};

#endif //! SOT_MOTION_PLANNER_ERROR_ESTIMATOR_HH
