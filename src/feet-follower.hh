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

#ifndef SOT_MOTION_PLANNER_FEET_FOLLOWER_HH
# define SOT_MOTION_PLANNER_FEET_FOLLOWER_HH
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

# include "common.hh"
# include "discretized-trajectory.hh"
# include "feet-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

class FeetFollower;

sot::MatrixHomogeneous
transformPgFrameIntoAnkleFrame (double tx, double ty, double tz, double theta,
				const sot::MatrixHomogeneous& feetToAnkle);

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  class Start : public Command
  {
  public:
    Start (FeetFollower& entity, const std::string& docstring);
    virtual Value doExecute();
  };
}


class FeetFollower : public dg::Entity
{
 public:
  typedef dg::SignalTimeDependent<ml::Vector, int> signalCoM_t;
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signalFoot_t;

  explicit FeetFollower (const std::string& name);
  virtual ~FeetFollower ();

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

  void start ()
  {
    started_ = true;
  }

protected:
  void update (int t)
  {
     if (t <= t_)
       return;
    t_ = t;
    impl_update ();
  }

  virtual void impl_update () = 0;

  int t_;
  ml::Vector com_;
  ml::Vector zmp_;
  sot::MatrixHomogeneous leftAnkle_;
  sot::MatrixHomogeneous rightAnkle_;

  double comZ_;
  sot::MatrixHomogeneous leftFootToAnkle_;
  sot::MatrixHomogeneous rightFootToAnkle_;
  sot::MatrixHomogeneous initialLeftAnklePosition_;
  sot::MatrixHomogeneous initialRightAnklePosition_;

  bool started_;

  signalCoM_t comOut_;
  signalCoM_t zmpOut_;
  signalFoot_t leftAnkleOut_;
  signalFoot_t rightAnkleOut_;


private:
  ml::Vector& updateCoM (ml::Vector& res, int t)
  {
    if (t > t_)
      update (t);
    res = com_;
    return res;
  }

  ml::Vector& updateZmp (ml::Vector& res, int t)
  {
    if (t > t_)
      update (t);
    res = zmp_;
    return res;
  }

  sot::MatrixHomogeneous& updateLeftAnkle (sot::MatrixHomogeneous& res, int t)
  {
    if (t > t_)
      update (t);
    res = leftAnkle_;
    return res;
  }

  sot::MatrixHomogeneous& updateRightAnkle (sot::MatrixHomogeneous& res, int t)
  {
     if (t > t_)
      update (t);
    res = rightAnkle_;
    return res;
  }

  void setComZ(const double& v)
  {
    comZ_ = v;
  }

  void setLeftFootToAnkle(const maal::boost::Matrix& v)
  {
    leftFootToAnkle_ = v;
  }

  void setRightFootToAnkle(const maal::boost::Matrix& v)
  {
    rightFootToAnkle_ = v;
  }

  void setInitialLeftAnklePosition(const maal::boost::Matrix& v)
  {
    initialLeftAnklePosition_ = v;
  }

  void setInitialRightAnklePosition(const maal::boost::Matrix& v)
  {
    initialRightAnklePosition_ = v;
  }
};

#endif //! SOT_MOTION_PLANNER_FEET_FOLLOWER_HH
