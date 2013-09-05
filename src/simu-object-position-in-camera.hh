// Copyright 2013,  Olivier Stasse,
// Gepetto, LAAS, CNRS.
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

#ifndef SOT_MOTION_PLANNER_SIMU_OBJECT_POSITION_IN_CAMERA_HH
# define SOT_MOTION_PLANNER_SIMU_OBJET_POSITION_IN_CAMERA_HH
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <sot/core/matrix-homogeneous.hh>

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

class SimuObjectPositionInCamera : public dg::Entity
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
  explicit SimuObjectPositionInCamera (const std::string& name);
  virtual ~SimuObjectPositionInCamera ();
  /// \}
  
  virtual void display(std::ostream & os) const;
protected:
  /// \brief Update position and timestamp.
  void update (int t);

  /// \brief Update the position signal.
  sot::MatrixHomogeneous&
  updateCamMo(sot::MatrixHomogeneous& res, int);

private:
  /// \brief Store the position of the object taken when the robot was at the starting point of its motion.
  sot::MatrixHomogeneous wpgMoinit_;
  
  /// \brief Store the position of the object in the camera reference frame.
  sot::MatrixHomogeneous camMo_;
  
  /// \brief Store the position of the camera in the world.
  sot::MatrixHomogeneous wMcam_;
  
  /// \brief Store the center of mass position in the world.
  sot::MatrixHomogeneous wMcom_;
  
  /// \brief Store the free flyer to be used to fill wMcom_ with some orientation.
  sot::MatrixHomogeneous freeFlyer_;

  /// \brief Store the center of mass position in the walking pattern generator reference frame (different from the world reference frame).
  sot::MatrixHomogeneous wpgMcom_;

  /// \name Signals
  /// @{

  /// \brief Static object position in the world PG.
  /// May change if we are restarting a walking sequence.
  signalMatrixHomoIn_t wpgMoinitIn_;

  /// \brief Camera position w.r.t. the world frame.
  signalMatrixHomoIn_t wMcamIn_;

  /// \brief CoM position w.r.t. the world frame.
  signalVectorIn_t wMcomIn_;

  /// \brief CoM position w.r.t. the world pattern generator frame.
  signalVectorIn_t wpgMcomIn_;  

  /// \brief CoM attitude w.r.t. the world frame.
  signalVectorIn_t wpgMcomAttIn_;
  
  /// \brief Object in Camera .
  signalMatrixHomoOut_t camMoOut_;
  /// @}

};

#endif //! SOT_MOTION_PLANNER_ROBOT_POSITION_FROM_VISP_HH
