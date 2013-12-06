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

#include "MotionCaptureToWaistMObject.hh"
#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>

// This program is used with the motion capture. It allowed to get the position of 
// an object in the waist frame from the motion capture information and the transformation
// matrix between the reflectors frame and the robot frame.
// 
// If you use this program with sway-motion-correction.cc, you need to delete in
// sway-motion-correction.cc the part where cMo is put in the waist frame because the output
// of this program give already the position of the object in the waist frame.

 //MotionCapture class 
MotionCapture::MotionCapture (const std::string& name)
  : dg::Entity (name),
    mcMrob_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "mcMrob")),
    mcMobj_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "mcMobj")),
    waistMo_ (INIT_SIGNAL_OUT
		       ("waistMo",
			MotionCapture::update, "MatrixHomo")),
	mcToObj_(),
	mcToRob_()
 {
  signalRegistration (mcMrob_ << mcMobj_ << waistMo_ );
  
  //initialize mcToObj_
  mcToObj_(0,0)=1.;
  mcToObj_(0,1)=0.;
  mcToObj_(0,2)=0.;
  mcToObj_(0,3)=0.;
  
  mcToObj_(1,0)=0.;
  mcToObj_(1,1)=1.;
  mcToObj_(1,2)=0.;
  mcToObj_(1,3)=0.;
  
  mcToObj_(2,0)=0.;
  mcToObj_(2,1)=0.;
  mcToObj_(2,2)=1.;
  mcToObj_(2,3)=0.;
  
  mcToObj_(3,0)=0.;
  mcToObj_(3,1)=0.;
  mcToObj_(3,2)=0.;
  mcToObj_(3,3)=1.;
                
  //Initialise mcToRob_;
  mcToRob_(0,0)=0.5033;
  mcToRob_(0,1)=0.8634;
  mcToRob_(0,2)=-0.0452;
  mcToRob_(0,3)=-0.1030;
  
  mcToRob_(1,0)=-0.8634;
  mcToRob_(1,1)=0.5043;
  mcToRob_(1,2)=0.0154;
  mcToRob_(1,3)=-0.0054;
  
  mcToRob_(2,0)=0.0361;
  mcToRob_(2,1)=0.0313;
  mcToRob_(2,2)=0.9989;
  mcToRob_(2,3)=0.2297;
  
  mcToRob_(3,0)=0.;
  mcToRob_(3,1)=0.;
  mcToRob_(3,2)=0.;
  mcToRob_(3,3)=1.;
  

  // Command           
  std::string docstring;
  addCommand
    ("setRobotTransformation",
     new command::motionCapture::SetRobotTransformation
     (*this, docstring));

  addCommand
    ("setObjectTransformation",
     new command::motionCapture::SetObjectTransformation
     (*this, docstring));

  // Dependencies
  waistMo_.addDependency ( mcMrob_ );
  waistMo_.addDependency ( mcMobj_ );
}     
     
MotionCapture::~MotionCapture ()
{
}

sot::MatrixHomogeneous&
MotionCapture::update(sot::MatrixHomogeneous& ret, int t) {
// Compute the homogeneous matrix of the object in the waist frame

 ret = mcToRob_ * mcMrob_(t).inverse() * mcMobj_(t) * mcToObj_.inverse();
 return ret;
}

void
MotionCapture::setRobotTransformation(ml::Matrix& mcToRob)
{
	for ( int i=0 ; i<4 ; i++ ) {
		for (int j=0 ; j<4 ; j++) {
			mcToRob_(i,j) = mcToRob(i,j);
		}
	}
}

void
MotionCapture::setObjectTransformation(ml::Matrix& mcToObj)
{
	for ( int i=0 ; i<4 ; i++ ) {
		for (int j=0 ; j<4 ; j++) {
            mcToObj_(i,j) = mcToObj(i,j);
        }
    }
}

 // Dynamic graph
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN
(MotionCapture, "MotionCapture");

namespace command
{
  namespace motionCapture
  {
    SetRobotTransformation::SetRobotTransformation (MotionCapture& entity,
							const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::MATRIX),
	 docstring)
    {}

    Value
    SetRobotTransformation::doExecute ()
    {
      MotionCapture& entity =
	static_cast<MotionCapture&> (owner ());
	std::vector<Value> values = getParameterValues ();
	ml::Matrix M = values[0].value ();
      entity.setRobotTransformation (M);
      return Value ();
    }

  SetObjectTransformation::SetObjectTransformation (MotionCapture& entity,
						const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::MATRIX),
	 docstring)
    {}

    Value
    SetObjectTransformation::doExecute ()
    {
      MotionCapture& entity =
	static_cast<MotionCapture&> (owner ());
     std::vector<Value> values = getParameterValues ();
     ml::Matrix M = values[0].value ();
       entity.setObjectTransformation(M);
       return Value ();
    } 
  }
}
