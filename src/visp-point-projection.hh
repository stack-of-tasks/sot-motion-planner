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

#ifndef SOT_MOTION_PLANNER_VISP_POINT_PROJECTION_HH
# define SOT_MOTION_PLANNER_VISP_POINT_PROJECTION_HH
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <sot/core/matrix-homogeneous.hh>

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

class VispPointProjection : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  /// \brief Input homogeneous matrix signal.
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;
  /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<ml::Vector, int> signalVectorOut_t;
  /// \brief Output double signal.
  typedef dg::SignalTimeDependent<double, int>
  signalDoubleOut_t;


  /// \name Constructor and destructor.
  /// \{
  explicit VispPointProjection (const std::string& name);
  virtual ~VispPointProjection ();
  /// \}

protected:
  /// \brief Update xy and z.
  void update (int t);

  ml::Vector& updateXy (ml::Vector& res, int t)
  {
    update (t);
    res = xy_;
    return res;
  }

  double& updateZ (double& res, int t)
  {
    update (t);
    res = z_;
    return res;
  }

private:
  ml::Vector xy_;
  double z_;

  /// \brief Object position in camera frame.
  signalMatrixHomoIn_t cMoIn_;

  /// \brief XY
  signalVectorOut_t xyOut_;
  /// \brief Z
  signalDoubleOut_t zOut_;
};

#endif //! SOT_MOTION_PLANNER_VISP_POINT_PROJECTION_HH
