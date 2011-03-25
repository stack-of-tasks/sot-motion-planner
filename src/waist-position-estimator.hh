// Copyright 2010, Duong Dang
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

#ifndef SOT_MOTION_PLANNER_POSITION_ESTIMATOR_HH
# define SOT_MOTION_PLANNER_POSITION_ESTIMATOR_HH
# include <string>

# include <boost/optional.hpp>

# include <jrl/mal/boost.hh>

# include <dynamic-graph/command-getter.h>
# include <dynamic-graph/command-setter.h>
# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/pool.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <sot/core/matrix-homogeneous.hh>
# include <dynamic-graph/signal-ptr.h>

# include <sot/core/matrix-homogeneous.hh>

class WaistPositionEstimator: public dynamicgraph::Entity
{
public:
  explicit WaistPositionEstimator (const std::string& name);
  virtual ~WaistPositionEstimator (){};
  static const std::string CLASS_NAME;

  dynamicgraph::SignalPtr<ml::Vector,int> forceLlegSIN;
  dynamicgraph::SignalPtr<ml::Vector,int> forceRlegSIN;
  dynamicgraph::SignalPtr<dynamicgraph::sot::MatrixHomogeneous,int> leftAnkleSIN;
  dynamicgraph::SignalPtr<dynamicgraph::sot::MatrixHomogeneous,int> rightAnkleSIN;
  dynamicgraph::SignalPtr<ml::Vector,int> stateSIN;
  dynamicgraph::SignalTimeDependent<double,int> zSOUT;
  dynamicgraph::SignalTimeDependent<ml::Vector,int> stateSOUT;
  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }
protected:
  double& computeZ(double& res, int time);
  ml::Vector& computeState(ml::Vector& res, int time);

};



#endif //! SOT_MOTION_PLANNER_POSITION_ESTIMATOR_HH
