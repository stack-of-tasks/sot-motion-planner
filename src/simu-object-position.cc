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

#include "simu-object-position.hh"
#include "common.hh"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

//const std::string SimuObjectInCam::CLASS_NAME="SimuObjectInCam";

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

  simu_cMo_.addDependency ( wMc_ );
}     
     
SimuObjectInCam::~SimuObjectInCam ()
{
}

void
SimuObjectInCam::initialize (int t)
{
	ciMo_ = cMo_(t);
	wMci_ = wMc_(t);
	
	initialized_ = true;
}

sot::MatrixHomogeneous&
SimuObjectInCam::update(sot::MatrixHomogeneous& ret, int t) {
	// Add protection if no initialization
	ret=wMc_(t).inverse()*wMci_*ciMo_;
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
}
}
