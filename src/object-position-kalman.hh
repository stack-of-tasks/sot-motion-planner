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
//#include <Eigen/Dense>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureTranslation.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpVelocityTwistMatrix.h>

#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

class CMoKalman : public dg::Entity
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
  explicit CMoKalman (const std::string& name)
  ;
  virtual ~CMoKalman ();
  /// \}

 /// \brief Update simu_cMo.
  sot::MatrixHomogeneous& update(sot::MatrixHomogeneous& ret, int t);
 /// \brief initialize first positions.
  void initialize(int t);
 /// \brief set Kalman parameter W V and P
  void setKalman(const double& p, const double& v, const double& w);

protected:    
  // Kalman parameters
  /// \brief X = (x, y , theta) state vector of Kalman filter
  vpColVector X1_;
  vpColVector X2_;
  
  /// \brief W noise on model
  vpMatrix W_;
  
  /// \brief V noise on measure
  vpMatrix V_;
  
  /// |brief P covariance of the Kalman Filter
  vpMatrix P_;
  
  /// \brief K gain of the Kalman filter
  vpMatrix K_;
  
  /// \brief Y_ measures
  vpColVector Y_;
  
 // Vision visp parameters
   /// \brief Translation feature handling position servoing.
  vpFeatureTranslation FT_;
  /// \brief Theta U feature handling orientation servoing.
  vpFeatureThetaU FThU_;

  /// \brief Task computing the control law.
  vpServo task_; 

 // Signals
 /// \brief cMo
  signalMatrixHomoIn_t cMo_;

 /// \brief COM velocity 
  signalVectorIn_t inputdcom_;

 /// \brief wMc
  signalMatrixHomoIn_t wMc_;
  
 /// \brief wMwaist
  signalMatrixHomoIn_t wMwaist_;
  
 /// |brief simulated cMo by Kalman
  signalMatrixHomoOut_t simu_cMo_;
  
 // Other parameters 
 /// \brief Is the first positions taken
  bool initialized_;
  
 /// \brief object height
  double Zi_;
};
  

  namespace command
{
  namespace cMoKalman
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class Initialize : public Command
    {
    public:
      Initialize (CMoKalman& entity,
		  const std::string& docstring);
      virtual Value doExecute ();
    };
    
    class SetKalman : public Command
    {
    public:
      SetKalman (CMoKalman& entity,
		  const std::string& docstring);
      virtual Value doExecute ();
    };
  
  } // end of namespace cMoKalman
} // end of namespace command.
