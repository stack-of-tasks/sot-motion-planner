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

/// \brief Compute the error between the planned and real robot
/// positions.
///
///
/// This entity compares:
///
/// - a planned position provided by the 'planned' signal as an
/// homogeneous matrix.
///
/// - a robot localization provided by the 'position' signal as a (x,
///   y, theta) vector.
///
/// The planned positions are bufferized into plannedPositions_ and tagged
/// with their execution time.
/// On the opposite, 'positionTimestamp' provides the acquisition time.
///
/// Additionnally, the planned position must be given in the world frame.
/// On the opposite, the localization data is given in the sensor frame.
///
/// The wMsensor_ attribute provides the transformation from sensor
/// frame to world frame.
///
/// The error is computed by:
///
/// wMestimated = wMsensor . sensorMestimated
/// wMplanned
///
/// estimatedMplanned = estimatedMw . wMplanned
/// estimatedMplanned = (wMestimated)^{-1} . wMplanned
///
///
/// wMlf = wMw' . w'Mlf
///
class ErrorEstimator : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
 public:
  /// \brief POSIX time
  typedef boost::posix_time::ptime ptime_t;

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
  explicit ErrorEstimator (const std::string& name);
  virtual ~ErrorEstimator ();
  /// \}

  /// \brief Set the reference trajectory.
  void setReferenceTrajectory (FeetFollower* ptr);

  /// \brief Update the error signal.
  ml::Vector& updateError (ml::Vector& res, int);

  sot::MatrixHomogeneous&
  updateDbgPositionWorldFrame (sot::MatrixHomogeneous& res, int);

  sot::MatrixHomogeneous&
  updateDbgPlanned (sot::MatrixHomogeneous& res, int);

  ml::Vector& updateDbgIndex (ml::Vector& res, int);

  sot::MatrixHomogeneous&
  updateDbgDeltaCommand (sot::MatrixHomogeneous& res, int);

  /// \brief Set the optional maximum error.
  void setSafetyLimits (const double& maxErrorX,
			const double& maxErrorY,
			const double& maxErrorTheta);
  /// \brief Remove the optional maximum error.
  void unsetSafetyLimits ();

protected:
  /// \brief Compute the index in the plannedPositions_ vector
  /// matching a given timestamp.
  size_t timestampToIndex (const ml::Vector& timestamp);

  /// \brief Set the sensor to world transformation.
  void sensorToWorldTransformation (const ml::Matrix& wMsensor)
  {
    wMsensor_ = wMsensor;
  }

  /// \brief Transform a coordinate from sensor frame to world frame.
  /// {}^w p = {}^w M_{sensor} {}^sensor p
  sot::MatrixHomogeneous wMsensor_;

  /// \brief Robot estimated position (X, Y, theta).
  signalVectorIn_t position_;
  /// \brief Robot estimated position timestamp (time)
  signalVectorIn_t positionTimestamp_;

  /// \brief Current planned position.
  signalMatrixHomoIn_t planned_;
  /// \brief Planned robot position in the real robot frame.
  signalVectorOut_t error_;

  /// \brief Planned comnand.
  signalVectorIn_t plannedCommand_;

  /// \brief Real comnand.
  ///
  /// This data can be given by the OpenHRP device to take into
  /// account perturbations introduced by the stabilizer.
  signalVectorIn_t realCommand_;

  /// \brief Jacobian of the reference point where error
  /// is computed.
  ///
  /// It is usually the current contact point (i.e. left-ankle).
  signalMatrixIn_t referencePointJacobian_;

  signalMatrixHomoOut_t dbgPositionWorldFrame_;
  signalMatrixHomoOut_t dbgPlanned_;
  signalVectorOut_t dbgIndex_;
  signalMatrixHomoOut_t dbgDeltaCommand_;

  sot::MatrixHomogeneous dbgPositionWorldFrameValue_;
  sot::MatrixHomogeneous dbgPlannedValue_;
  ml::Vector dbgIndexValue_;
  sot::MatrixHomogeneous dbgDeltaCommandValue_;

  /// \brief Pointer to the reference trajectory.
  FeetFollower* referenceTrajectory_;

  /// \brief Set of past planned positions (time, index, position).
  std::vector<boost::tuple<ptime_t, unsigned, sot::MatrixHomogeneous> >
  plannedPositions_;

  /// \brief Did the movement start?
  bool started_;

  /// \brief Optional maximum allowed error (x, y, theta).
  boost::optional<boost::array<double, 3> > maxError_;
};

#endif //! SOT_MOTION_PLANNER_ERROR_ESTIMATOR_HH
