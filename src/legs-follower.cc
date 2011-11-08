// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <limits>
#include <string>

#include <boost/format.hpp>
#include <boost/optional.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/date_time/date.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <jrl/mal/boost.hh>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-time-dependent.h>
#define VP_DEBUG
#define VP_DEBUG_MODE 40
#include <sot/core/debug.hh>

#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sys/time.h>


#include "common.hh"
#include "discretized-trajectory.hh"
#include "legs-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

static const double STEP = 0.005; //FIXME:

enum {WAIT = -1, READY = 1, RUN = 2, STOP = 3};

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

using ::dynamicgraph::command::Setter;

LegsFollower::LegsFollower (const std::string& name)
  : Entity(name),
    t_ (std::numeric_limits<int>::min ()),
    lastID (-1.0),
    started_ (0),
    startTime_ (-1.),
    startIndex_ (-1),
    ldof_out_ (INIT_SIGNAL_OUT ("ldof", LegsFollower::update_ldof, "Vector")),
    waist_out_ (INIT_SIGNAL_OUT ("waist", LegsFollower::update_waist, "MatrixHomogeneous")),
    comOut_ (INIT_SIGNAL_OUT ("com", LegsFollower::update_com, "Vector")),
    zmpOut_ (INIT_SIGNAL_OUT ("zmp", LegsFollower::update_zmp, "Vector")),
    inputRef_   (dg::nullptr,MAKE_SIGNAL_STRING (name, true, "Vector", "inputRef")),
    outputStart_   (dg::nullptr,MAKE_SIGNAL_STRING (name, true, "Vector", "outputStart")),
    outputYaw_   (dg::nullptr,MAKE_SIGNAL_STRING (name, true, "Vector", "outputYaw"))
{
  sotDEBUGIN(15);
  signalRegistration (ldof_out_ << waist_out_ << comOut_ << zmpOut_ << inputRef_ << outputStart_ << outputYaw_);

  ldof_out_.setNeedUpdateFromAllChildren (true);
  comOut_.setNeedUpdateFromAllChildren (true);
  zmpOut_.setNeedUpdateFromAllChildren (true);
  waist_out_.setNeedUpdateFromAllChildren (true);

  ml::Vector temp(3);
  double sec, usec;
  getAbsoluteTime(sec,usec);
  temp(0) = WAIT;
  temp(1) = sec;
  temp(2) = usec;
  outputStart_ = temp;

  
  std::string docstring;

  addCommand ("start", new command::legsFollower::Start (*this, docstring));
  addCommand ("stop", new command::legsFollower::Stop (*this, docstring));

  buffer.resize(5000000,0.0);
  std::cout << "Buffer created with 5.000.000 doubles." << buffer.size() << std::endl;
  ml::Vector yaw;
  yaw.resize(1);
  
  outputYaw_ = yaw;
  sotDEBUGOUT(15);
}

LegsFollower::~LegsFollower ()
{
  buffer.clear();
}

void LegsFollower::getAbsoluteTime(double& sec, double& usec)
{
  sotDEBUGIN(15);
#if 1
  boost::posix_time::ptime t(boost::posix_time::microsec_clock::universal_time ());
  boost::posix_time::time_duration t_now = t.time_of_day();
  sec = t_now.total_seconds();
  double total_usec = (double) t_now.total_microseconds();
  usec = total_usec - 1000000.0*sec;
#else
  struct timeval tv;
  gettimeofday (&tv,NULL);
  sec = tv.tv_sec;
  usec = tv.tv_usec;
#endif
  sotDEBUGOUT(15);
}


void
LegsFollower::start ()
{
  sotDEBUGIN(15);
  updateRefFromCorba();

  ml::Vector temp(3);
  double sec, usec;
  getAbsoluteTime(sec,usec);

  if(lastID < 0.0){
    started_ = READY; //ready to start
    //std::cout << "Start pressed but no message have been sent." << endl;

    temp(0) = READY;
    temp(1) = sec;
    temp(2) = usec;
    outputStart_ = temp;
    return;
  }

  startIndex_ = t_;
  startTime_ = t_ * STEP;
  impl_start ();

  
  temp(0) = RUN;
  temp(1) = sec;
  temp(2) = usec;
  outputStart_ = temp;
  started_ = RUN;
  std::cout << "Start at index "<< startIndex_ << " (" << startTime_ << "s)" << std::endl;
  sotDEBUGOUT(15);

}

void
LegsFollower::stop()
{
  sotDEBUGIN(15);
  ml::Vector temp(3);
  double sec, usec;
  getAbsoluteTime(sec,usec);

  temp(0) = STOP;
  temp(1) = sec;
  temp(2) = usec;

  outputStart_ = temp;
  started_ = STOP;
  sotDEBUGOUT(15);
}

void
LegsFollower::update (int t)
{
  sotDEBUGIN(15);
  if (t <= t_)
    return;
  t_ = t;

  if(started_ == 1){
    start();
  }

  //Test end of path
  if(started_ == RUN || started_ == STOP)
    {
      updateRefFromCorba();

      //FIXME : stop at the end of trajectory
      if(t_ >= startIndex_ + endOfMessage*200 - 1){
	if(t_ < startIndex_ + endOfPath*200 - 1){
	  std::cout << "startIndex_ : " << startIndex_ << " current : " << t_ << "." << std::endl;
	  std::cout << "Prematured end of path : " << endOfMessage << " instead " << endOfPath << "." << std::endl;
	  exit(13);
	}

	t_ = startIndex_ + (int)(endOfMessage*200) - 1;
      }
    }

  //Update Yaw :
  ml::Vector yaw;
  yaw.resize(1);

  if(started_ == RUN || started_ == STOP){
    int index = t_-startIndex_;
    yaw(0) = buffer[17*(index) + 12 + 2];
  }else{
    yaw(0) = 0.0;
  }
  outputYaw_ = yaw;
  sotDEBUGOUT(15);
}


ml::Vector&
LegsFollower::update_ldof (ml::Vector& res, int t)
{
  sotDEBUGIN(15);
  if (t > t_)
    update (t);

  res.resize(12);

  int index;

  if(started_ == RUN || started_ == STOP){
    index = t_-startIndex_;
    for(int k=0; k< 12; ++k){
      res(k) = buffer[17*(index) + k];
    }
  }else{
    //half-sitting
    //double pos[12] = {0, 0, -0.453786, 0.872665, -0.418879, 0, 0, 0, -0.453786, 0.872665, -0.418879, 0};
    double pos[12] = {-0, 0, -0.451027, 0.902054, -0.451027, 0, 0, 0,-0.451027, 0.902054, -0.451027, 0};
    for(int k=0; k< 12; ++k){
      res(k) = pos[k];
    }
  }
  sotDEBUGOUT(15);
  return res;
}

sot::MatrixHomogeneous&
LegsFollower::update_waist (sot::MatrixHomogeneous& res, int t)
{
  //printf("update_ldof\n");
  sotDEBUGIN(15);
  if (t > t_)
    update (t);

  if(started_ == RUN || started_ == STOP){
    int index = t_-startIndex_;

    while(buffer[17*index + 12 + 2] >  M_PI) buffer[17*index + 12 + 2] -= 2*M_PI;
    while(buffer[17*index + 12 + 2] < -M_PI) buffer[17*index + 12 + 2] += 2*M_PI;

    ml::Vector xyztheta (4);
    xyztheta (0) = buffer[17*index + 12 + 0];
    xyztheta (1) = buffer[17*index + 12 + 1];
    xyztheta (2) = 0.65;
    xyztheta (3) = buffer[17*index + 12 + 2];

    res =  XYZThetaToMatrixHomogeneous (xyztheta);
  }else{
    ml::Vector xyztheta (4);
    xyztheta (0) = 0.0;
    xyztheta (1) = 0.0;
    xyztheta (2) = 0.65;
    xyztheta (3) = 0.0;

    res =  XYZThetaToMatrixHomogeneous (xyztheta);
  }
  sotDEBUGOUT(15);
  return res;
}

ml::Vector&
LegsFollower::update_com (ml::Vector& res, int t)
{
  //printf("update_com\n");
  sotDEBUGIN(15);
  if (t > t_)
    update (t);

  res.resize(3);

  if(started_ == RUN || started_ == STOP){
    int index = t_-startIndex_;
    for(int k=0; k< 2; ++k){
      res(k) = buffer[17*index + 12 + k];
    }
  }else{
    res(0) = 0.0;
    res(1) = 0.0;
  }

  res(2) = 0.814;
  sotDEBUGOUT(15);
  return res;
}

ml::Vector&
LegsFollower::update_zmp (ml::Vector& res, int t)
{
  sotDEBUGIN(15);
  if (t > t_)
    update (t);

  res.resize(3);

  if(started_ == RUN || started_ == STOP){
    int index = t_-startIndex_;
    for(int k=0; k< 2; ++k){
      res(k) = buffer[17*index + 15 + k];
    }
    res(2) = 0.0;
  }else{
    for(int k=0; k< 3; ++k){
      res(k) = 0.0;
    }
  }
  sotDEBUGOUT(15);
  return res;
}


void
LegsFollower::updateRefFromCorba()
{
  sotDEBUGIN(15);
  // message : { ID, end of path, beginning of modification, end of modification,  Values ... }
  ml::Vector r = inputRef_ (0);

  if(r.size()<4){
    //std::cout << "Vector is too small.\n"; //<< std::flush;
    return;
  }

  if( r(0) < lastID + 0.5){
    //printf("No new message.\n");
    return;
  }

  if( r(0) > lastID + 1.5){
    std::cout << "Warning : One message was lost !!! (jump from " << lastID << " to " << r(0) << ")" << std::endl;
    exit(10);
  }

  lastID = r(0);
  endOfPath = r(1);
  endOfMessage = r(3);

  int begin = (int) (r(2)/0.005 + 0.001);
  int end   = (int) (r(3)/0.005 + 0.001);

  std::cout << t_ << " New message : ID "<< lastID << " with " << r.size() << " values, from " << begin*0.005 << " to " << end*0.005 << ", end of path = "<< endOfPath <<"."<< std::endl;

  int currIndex = t_-startIndex_;
  if( (started_ == RUN || started_ == STOP) && currIndex > begin){
    std::cout << "Warning : Modification of current step (currentIndex=" <<currIndex << ", begin=" << begin << ")." << std::endl;
    exit(11);
  }

  if( end*17 >= (int) buffer.size() ){
    std::cout << "Buffer is too small." << std::endl;
    exit(12);
    return;
  }

  for (int i = 0; i < (end-begin)*17; ++i){
    buffer[begin*17 + i] = r(4+i);
  }
  sotDEBUGOUT(15);
}

double
LegsFollower::getTime () const
{
  return t_ * STEP;
}

double
LegsFollower::getTrajectoryTime () const
{
  if (!started_)
    return 0.;
  return getTime () - startTime_;
}

double
LegsFollower::startTime () const
{
  return startTime_;
}

namespace command
{
  namespace legsFollower
  {
    Start::Start (LegsFollower& entity, const std::string& docstring)
      : Command (entity, std::vector<Value::Type> (), docstring)
    {}

    Value Start::doExecute()
    {
      LegsFollower& entity = static_cast<LegsFollower&>(owner ());
      entity.start ();
      return Value ();
    }

    Stop::Stop (LegsFollower& entity, const std::string& docstring)
      : Command (entity, std::vector<Value::Type> (), docstring)
    {}

    Value Stop::doExecute()
    {
      LegsFollower& entity = static_cast<LegsFollower&>(owner ());
      entity.stop ();
      return Value ();
    }
  }
} // end of namespace command.


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (LegsFollower, "LegsFollower");

