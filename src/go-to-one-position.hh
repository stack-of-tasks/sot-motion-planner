// Copyright 2013,  Olivier Stasse.
// Gepetto, LAAS CNRS
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

#ifndef SOT_MOTION_PLANNER_GO_TO_ONE_POSITION_HH
# define SOT_MOTION_PLANNER_GO_TO_ONE_POSITION_HH
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <sot/core/matrix-homogeneous.hh>

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

class GoToOnePosition : public dg::Entity
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
  explicit GoToOnePosition (const std::string& name);
  virtual ~GoToOnePosition();
  /// \}
  
  /// \brief Set the gains for the velocity
  /// 
  void setGains(const ml::Vector & gains)
  {
    gains_ = gains;
  }

  /// \brief Set the upper for the velocity
  void setUpperLimit(const ml::Vector & limits_up)
  {
    limits_up_ = limits_up;
  }

  /// \brief Set the upper for the velocity
  void setBottomLimit(const ml::Vector & limits_bottom)
  {
    limits_bottom_ = limits_bottom;
  }

  /// \brief Set the convergence boolean
  void setConvergence(const std::string &Value)
  {
    if (Value=="true")
      ConvergenceReached_ = false;
    if (Value=="false")
      ConvergenceReached_ = true;
  }

  friend std::ostream & operator << (std::ostream &os, const GoToOnePosition & ag21p);

protected:

  /// \brief Udpate computation of pgVelocity according
  /// to current robot position and target position.
  void update (int t);

  /// \brief Update the position signal.
  ml::Vector& updatePgVelocity (ml::Vector& res, int t);
  /// \brief Update the position signal.
  ml::Vector& updatePgVelocityTimestamp (ml::Vector& res, int t);

  /// \brief Display informations on a vector of go21position.
  void display_vector(std::string, std::ostream &, const ml::Vector &) const;
  
  /// \brief Display information on go21position.
  void display(std::ostream &os) const;
  
private:

  /// \brief Bool for convergence.
  bool ConvergenceReached_;

  /// \brief Robot Position.
  ml::Vector robotPosition_;
  
  /// \brief Target position
  ml::Vector targetPosition_;

  /// \brief Pg Velocity 
  ml::Vector pgVelocity_;

  /// \brief Gains for the velocity.
  ml::Vector gains_;

  /// \brief Upper limits for the velocity.
  ml::Vector limits_up_;
  
/// \brief Bottom limits for the velocity.
  ml::Vector limits_bottom_;

  /// @{
  /// \brief Robot waist position estimation.
  signalVectorIn_t robotPositionIn_;

  /// \brief Target waist position.
  signalVectorIn_t targetPositionIn_;
  ///@}
  
  /// \brief Pattern generator desired velocity to reach the target waist position.
  signalVectorOut_t pgVelocityOut_;

};

#endif //! SOT_MOTION_PLANNER_ROBOT_POSITION_FROM_VISP_HH
