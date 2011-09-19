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

#ifndef SOT_MOTION_PLANNER_ROBOT_POSITION_FROM_VISP_HH
# define SOT_MOTION_PLANNER_ROBOT_POSITION_FROM_VISP_HH
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <sot/core/matrix-homogeneous.hh>

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

class RobotPositionFromVisp : public dg::Entity
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
  explicit RobotPositionFromVisp (const std::string& name);
  virtual ~RobotPositionFromVisp ();
  /// \}

protected:
  /// \brief Update position and timestamp.
  void update (int t);

  /// \brief Update the position signal.
  ml::Vector& updatePosition (ml::Vector& res, int t);
  /// \brief Update the position signal.
  ml::Vector& updatePositionTimestamp (ml::Vector& res, int t);

  sot::MatrixHomogeneous&
  updateDbgcMo (sot::MatrixHomogeneous& res, int);
  sot::MatrixHomogeneous&
  updateDbgPosition (sot::MatrixHomogeneous& res, int);

  /// \brief Set the sensor to world transformation.
  ///
  /// Optional: if the standard frame orientation is not respected (X
  /// forward, Y left, Z up) is not respected by the tracker, this
  /// transformation can be used to fix the frame orientation.
  ///
  /// This is necessary when using the VpMbtEdgeTracker.
  void sensorTransformation (const ml::Matrix& cMc)
  {
    cMc_ = cMc;
  }


private:
  sot::MatrixHomogeneous cMc_;
  ml::Vector position_;
  ml::Vector positionTimestamp_;
  sot::MatrixHomogeneous dbgcMo_;
  sot::MatrixHomogeneous dbgPosition_;

  /// \brief Object position in camera frame (tracking data).
  signalMatrixHomoIn_t cMoIn_;
  /// \brief  Tracking data timestamp.
  signalVectorIn_t cMoTimestampIn_;

  /// \brief Current object position in world frame.
  signalMatrixHomoIn_t plannedObjectPositionIn_;

  /// \brief Robot waist position estimation.
  signalVectorOut_t positionOut_;
  /// \brief Robot waist position estimation (associated timestamp).
  signalVectorOut_t positionTimestampOut_;

  signalMatrixHomoOut_t dbgcMoOut_;
  signalMatrixHomoOut_t dbgPositionOut_;
};

#endif //! SOT_MOTION_PLANNER_ROBOT_POSITION_FROM_VISP_HH
