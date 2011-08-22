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

#ifndef SOT_MOTION_PLANNER_VIRTUAL_SENSOR_HH
# define SOT_MOTION_PLANNER_VIRTUAL_SENSOR_HH
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <sot/core/matrix-homogeneous.hh>

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

class VirtualSensor : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  /// \brief Input vector signal.
  typedef dg::SignalPtr<ml::Vector, int> signalVectorIn_t;
  /// \brief Input homogeneous matrix signal.
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;
  /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<ml::Vector, int> signalVectorOut_t;
  /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int>
  signalMatrixHomoOut_t;


  /// \name Constructor and destructor.
  /// \{
  explicit VirtualSensor (const std::string& name);
  virtual ~VirtualSensor ();
  /// \}

protected:
  /// \brief Update the position signal.
  ml::Vector& updatePosition (ml::Vector& res, int);
  /// \brief Update the position signal.
  ml::Vector& updatePositionTimestamp (ml::Vector& res, int);

private:
  signalMatrixHomoIn_t plannedIn_;
  signalMatrixHomoIn_t expectedObstaclePositionIn_;
  signalMatrixHomoIn_t obstaclePositionIn_;
  signalVectorOut_t positionOut_;
  signalVectorOut_t positionTimestampOut_;
};

#endif //! SOT_MOTION_PLANNER_VIRTUAL_SENSOR_HH
