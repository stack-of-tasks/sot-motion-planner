// Copyright 2011
// LAAS-CNRS, Mathieu Geisert 
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

#include "simu-object-position.hh"
#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

// This program give the position where the object should be according to a reference position
// and the odometry of the robot. The reference position is the initial position then it is updated
// if the difference between the calculated position and the mesured posiiton doesn't exceed a threshold.
SimuObjectInCam::SimuObjectInCam (const std::string& name)
  : dg::Entity (name),
    initialized_ (false),
    cMo_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "cMo")),
    wMc_ (dg::nullptr,
	  MAKE_SIGNAL_STRING
	  (name, true, "MatrixHomo", "wMc")),
	simu_cMo_ (INIT_SIGNAL_OUT
		       ("simu_cMo",
			SimuObjectInCam::update, "MatrixHomo")) {
  signalRegistration (cMo_ << wMc_ << simu_cMo_ );
  
  std::string docstring;
    
  addCommand
    ("initialize",
     new command::simuObjectInCam::Initialize
     (*this, docstring));
     
     // Command to set threshold (if the reference position is update or not)
  addCommand
    ("SetUDThreshold",
     new command::simuObjectInCam::SetUDThreshold
     (*this, docstring));
     
     // Command to set at wich frequency the reference position will be updated
  addCommand
    ("SetUDFrequency",
     new command::simuObjectInCam::SetUDFrequency
     (*this, docstring)); 
     
  IncUD_ = 0;
  
  UDThreshold_ = 0.2;
  UDFrequency_ = 199;
  
  simu_cMo_.addDependency ( wMc_ );
  simu_cMo_.addDependency ( cMo_ );
}     
     
SimuObjectInCam::~SimuObjectInCam ()
{
}

void 
SimuObjectInCam::setFrequency (int t)
{
	UDFrequency_ = t;
}

void 
SimuObjectInCam::setThreshold (double t)
{
	UDThreshold_ = t;
}


// initialization = set the reference position on the initial position
void
SimuObjectInCam::initialize (int t)
{
	ciMo_ = cMo_(t);
	wMci_ = wMc_(t);
	
	sot::MatrixHomogeneous wMo = wMc_(t)*cMo_(t);
	// We suppose the robot move on a flat floor so the position on Z axis = const
	Zi_  = wMo(2,3);
	
	initialized_ = true;
}

sot::MatrixHomogeneous&
SimuObjectInCam::update(sot::MatrixHomogeneous& ret, int t) {
	// Add protection if no initialization
	sot::MatrixHomogeneous cMo = wMc_(t).inverse()*wMci_*ciMo_;
	
	// Keep the same object height (in the world)
	sot::MatrixHomogeneous wMo = wMc_(t)*cMo;
	wMo(2,3) = Zi_;
	ret = wMc_(t).inverse()*wMo;
	
	// each second, if the cMo from visp ~ cMo simulated, up-date initial positions here
	if ( IncUD_ > UDFrequency_ ) {
	    bool CanUD=true;
		for (int i=0 ; i<2 ; i++) {
			if ( ret(i,3) - (cMo_(t))(i,3) > UDThreshold_ || ret(i,3) - (cMo_(t))(i,3) < -UDThreshold_ ) {
				CanUD = false;
			}
		}
		if ( CanUD == true ) {
			initialize(t);
			IncUD_ = 0;
		}
	}
	IncUD_++;			
	
	return ret;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN
(SimuObjectInCam, "SimuObjectInCam");

namespace command
{
  namespace simuObjectInCam
  {
    Initialize::Initialize (SimuObjectInCam& entity,
			    const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::INT),
	 docstring)
    {}

    Value
    Initialize::doExecute ()
    {
      SimuObjectInCam& entity =
	static_cast<SimuObjectInCam&> (owner ());
	std::vector<Value> values = getParameterValues ();
	int t = values[0].value ();
      entity.initialize (t);
      return Value ();
    }

    SetUDThreshold::SetUDThreshold (SimuObjectInCam& entity,
			    const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::DOUBLE),
	 docstring)
    {}

    Value
    SetUDThreshold::doExecute ()
    {
      SimuObjectInCam& entity =
	static_cast<SimuObjectInCam&> (owner ());
	std::vector<Value> values = getParameterValues ();
	double t = values[0].value ();
      entity.setThreshold (t);
      return Value ();
    }

    SetUDFrequency::SetUDFrequency (SimuObjectInCam& entity,
			    const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::INT),
	 docstring)
    {}

    Value
    SetUDFrequency::doExecute ()
    {
      SimuObjectInCam& entity =
	static_cast<SimuObjectInCam&> (owner ());
	std::vector<Value> values = getParameterValues ();
	int t = values[0].value ();
      entity.setFrequency (t);
      return Value ();
    }
}
}
