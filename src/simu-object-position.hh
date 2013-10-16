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

//#ifndef SOT_MOTION_PLANNER_SIMU_OBJECT_POSITION_HH
//# define SOT_MOTION_PLANNER_SIMU_OBJECT_POSITION_HH
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

class SimuObjectInCam : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
 public:
 /// \brief Input homogeneous matrix signal.
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;   
 /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signalMatrixHomoOut_t;
  
   /// \name Constructor and destructor.
  /// \{
  explicit SimuObjectInCam (const std::string& name);
  virtual ~SimuObjectInCam ();
  /// \}

 /// \brief Update simu_cMo.
  sot::MatrixHomogeneous& update(sot::MatrixHomogeneous& ret, int t);
 /// \brief initialize first positions.
  void initialize(int t);

protected:    
 /// \brief cMo
  signalMatrixHomoIn_t cMo_;
 /// \brief wMc
  signalMatrixHomoIn_t wMc_;
 /// \brief SimucMo
  signalMatrixHomoOut_t simu_cMo_;
  
 /// \brief first object position in camera 
  sot::MatrixHomogeneous ciMo_;
 /// \brief first camera position in world
  sot::MatrixHomogeneous wMci_; 
 
 /// \brief Is the first positions taken
  bool initialized_;
};
  
  namespace command
{
  namespace simuObjectInCam
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class Initialize : public Command
    {
    public:
      Initialize (SimuObjectInCam& entity,
		  const std::string& docstring);
      virtual Value doExecute ();
    };
  } // end of namespace simuObjectInCam
} // end of namespace command.
