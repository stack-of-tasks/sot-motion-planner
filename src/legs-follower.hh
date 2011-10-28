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

#ifndef SOT_MOTION_PLANNER_LEGS_FOLLOWER_HH
# define SOT_MOTION_PLANNER_LEGS_FOLLOWER_HH
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
#include <dynamic-graph/signal-ptr.h>
# include <sot/core/matrix-homogeneous.hh>

# include "common.hh"
# include "discretized-trajectory.hh"
//# include "feet-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

class LegsFollower;

sot::MatrixHomogeneous
transformPgFrameIntoAnkleFrame (double tx, double ty, double tz, double theta,
				const sot::MatrixHomogeneous& feetToAnkle);

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;
  
  namespace legsFollower
  {
    class Start : public Command
    {
    public:
      Start (LegsFollower& entity, const std::string& docstring);
      virtual Value doExecute();
    };
    class Stop : public Command
    {
    public:
      Stop (LegsFollower& entity, const std::string& docstring);
      virtual Value doExecute();
    };
  }
}


class LegsFollower : public dg::Entity
{
 public:

  DYNAMIC_GRAPH_ENTITY_DECL ();
  
  typedef dg::SignalTimeDependent<ml::Vector, int> signalCoM_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalZMP_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signal_ldof_t;
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signal_waist_t;
  typedef dg::SignalPtr<ml::Vector, int> signalIn_t;
  typedef dg::SignalPtr<ml::Vector, int> signalOut_t;

  explicit LegsFollower (const std::string& name);
  virtual ~LegsFollower ();


  void start ();
  void stop();

  void update (int t);
  ml::Vector& update_ldof (ml::Vector& res, int t);
  sot::MatrixHomogeneous& update_waist(sot::MatrixHomogeneous& res, int t);
  ml::Vector& update_com (ml::Vector& res, int t);
  ml::Vector& update_zmp (ml::Vector& res, int t);

  double getTime () const;
  void getAbsoluteTime(double& sec, double& usec);

  /// \brief Returns the current time index on the trajectory.
  /// I.e. time - startTime (0 means the trajectory replay just began)
  double getTrajectoryTime () const;


  virtual void impl_start ()
  { }

  signal_ldof_t& ldof_out ()
  {
    return ldof_out_;
  }

  signal_waist_t& waist_out ()
  {
    return waist_out_;
  }

  signalCoM_t& comOut ()
  {
    return comOut_;
  }

  signalZMP_t& zmpOut ()
  {
    return zmpOut_;
  }


  double startTime () const;

protected:
  virtual void impl_update ()
  { }

  void updateRefFromCorba();

  int t_;

  double lastID;
  double endOfPath;
  double endOfMessage;

  //Buffer : [ 12 articular values, CoM.x, CoM.y, Waist.yaw, zmp.x, zmp.y, ... ]
  std::vector<double> buffer;
  //boost::circular_buffer<double> buffer;

  int started_;
  double startTime_;
  int startIndex_;

  signal_ldof_t ldof_out_;
  signal_waist_t waist_out_;
  signalCoM_t comOut_;
  signalZMP_t zmpOut_;
  signalIn_t inputRef_;
  signalOut_t outputStart_;
  signalOut_t outputYaw_;

};

#endif //! SOT_MOTION_PLANNER_LEGS_FOLLOWER_HH
