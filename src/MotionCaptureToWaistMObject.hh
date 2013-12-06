// Copyright 2011
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


#include <boost/foreach.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot/core/vector-roll-pitch-yaw.hh>
#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

class MotionCapture : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
 public:
 /// \brief Input homogeneous matrix signal.
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;   
 /// \brief Output homogeneous matrix signal.
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signalMatrixHomoOut_t;
    /// \brief Input vector signal.
  typedef dg::SignalPtr<ml::Vector, int> signalVectorIn_t;
  
   /// \name Constructor and destructor.
  /// \{
  explicit MotionCapture (const std::string& name)
  ;
  virtual ~MotionCapture ();
  /// \}

 /// \brief Update waistMo.
  sot::MatrixHomogeneous& update(sot::MatrixHomogeneous& ret, int t);
 /// \brief set Transformation from the reflectors on the object frame to the object frame in the wrl file 
  void setObjectTransformation(ml::Matrix& mcToObj);
  /// \brief set Transformation from the reflectors on the robot frame to the waist frame
  void setRobotTransformation(ml::Matrix& mcToRob);
  

protected:    
 // Transformation matrix from motion capture to robot
 /// \brief mcToObj_ Transformation from the object frame in the motion capture to the object frame in the wrl file
 sot::MatrixHomogeneous mcToObj_;
 
 /// \brief mcToRob_ Transformation from the robot frame in the motion capture to the ??? frame in the robot
 sot::MatrixHomogeneous mcToRob_;
 
 // Signals
 /// \brief cmMrob_ Homogeneous matrix of the frame of the reflector on the robot in the motion capture frame
  signalMatrixHomoIn_t mcMrob_;
  
 /// \brief cmMobj_ Homogeneous matrix of thte frame of the reflector on the object in the motion capture frame
  signalMatrixHomoIn_t mcMobj_;
  
 /// |brief waistMo Homogeneous matrix of the frame of the object in the waist frame
  signalMatrixHomoOut_t waistMo_; 
};
  

  namespace command
{
  namespace motionCapture
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class SetRobotTransformation : public Command
    {
    public:
      SetRobotTransformation (MotionCapture& entity,
		  const std::string& docstring);
      virtual Value doExecute ();
    };
    
    class SetObjectTransformation : public Command
    {
    public:
      SetObjectTransformation (MotionCapture& entity,
		  const std::string& docstring);
      virtual Value doExecute ();
    };
  
  } // end of namespace MotionCapture
} // end of namespace command.
