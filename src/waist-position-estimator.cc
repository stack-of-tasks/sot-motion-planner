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

#include "waist-position-estimator.hh"
#include <sot-dynamic/dynamic.h>

#include "common.hh"
namespace sot
{
  using namespace ::dynamicgraph::sot;
}

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;


WaistPositionEstimator::WaistPositionEstimator(const std::string& name)
  : dynamicgraph::Entity(name)
  ,forceLlegSIN (NULL,"sotWaistPositionEstimator("+name+")::input(vector)::forceLleg")
  ,forceRlegSIN (NULL,"sotWaistPositionEstimator("+name+")::input(vector)::forceRleg")
  ,leftAnkleSIN (NULL,"sotWaistPositionEstimator("+name+")::input(homo)::leftAnkle")
  ,rightAnkleSIN(NULL,"sotWaistPositionEstimator("+name+")::input(homo)::rightAnkle")
  ,stateSIN     (NULL,"sotWaistPositionEstimator("+name+")::input(vector)::stateIn")
  ,zSOUT( boost::bind(&WaistPositionEstimator::computeZ,this,_1,_2),
          forceLlegSIN << forceRlegSIN
          << leftAnkleSIN << rightAnkleSIN << stateSIN,
          "sotWaistPositionEstimator("+name+")::output(double)::z" )
  ,stateSOUT( boost::bind(&WaistPositionEstimator::computeState,this,_1,_2),
              forceLlegSIN << forceRlegSIN
              << leftAnkleSIN << rightAnkleSIN << stateSIN,
              "sotWaistPositionEstimator("+name+")::output(double)::stateOut" )

{
  signalRegistration( forceLlegSIN << forceRlegSIN
                      << leftAnkleSIN << rightAnkleSIN << stateSIN
                      << zSOUT << stateSOUT);
}



double&
WaistPositionEstimator::computeZ(double &res, int time)
{
  ml::Vector forceL = forceLlegSIN(time-1);
  ml::Vector forceR = forceRlegSIN(time-1);
  ml::Vector stateIn = stateSIN(time);
  sot::MatrixHomogeneous leftAnkle = leftAnkleSIN(time-1);
  sot::MatrixHomogeneous rightAnkle = rightAnkleSIN(time-1);
  //const double EPSILON = 1e-6;
  const double CONTACT_FORCE_THRESHOLD = 200;
  const double ANKLE_HEIGHT = 0.105;
  // if lleg on the ground
  res = stateIn(2);

  if (forceL(2) > CONTACT_FORCE_THRESHOLD)
    {
      res -= ANKLE_HEIGHT - leftAnkle(2,3);
      return res;
    }
  if (forceR(2) > CONTACT_FORCE_THRESHOLD)
    {
      res -= ANKLE_HEIGHT - rightAnkle(2,3);
      return res;
    }
  return res;
}

ml::Vector&
WaistPositionEstimator::computeState(ml::Vector &res, int time)
{
  ml::Vector forceL = forceLlegSIN(time-1);
  ml::Vector forceR = forceRlegSIN(time-1);
  sot::MatrixHomogeneous leftAnkle = leftAnkleSIN(time-1);
  sot::MatrixHomogeneous rightAnkle = rightAnkleSIN(time-1);
  ml::Vector state = stateSIN(time);
  res.resize(state.size());
  for (unsigned i = 0; i < state.size(); i++)
    {
      res(i) = state(i);
    }

  //const double EPSILON = 1e-6;
  const double CONTACT_FORCE_THRESHOLD = 200;
  const double ANKLE_HEIGHT = 0.105;
  // if lleg on the ground
  if (forceL(2) > CONTACT_FORCE_THRESHOLD)
    {
      res(2) -= ANKLE_HEIGHT - leftAnkle(2,3);
      return res;
    }
  if (forceR(2) > CONTACT_FORCE_THRESHOLD)
    {
      res(2) -= ANKLE_HEIGHT - rightAnkle(2,3);
      return res;
    }
  return res;
}


DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WaistPositionEstimator,
                                   "WaistPositionEstimator");
