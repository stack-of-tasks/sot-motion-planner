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


/// \brief Gathers information required to execute a walking movement.
///
/// This structure gathers trajectories of the left ankle, right ankle
/// and center of mass.
struct WalkMovement
{
  /// \brief Walk phases.
  enum SupportFoot
    {
      /// \brief Double support.
      SUPPORT_FOOT_DOUBLE = -1,
      /// \brief Left foot is support foot.
      SUPPORT_FOOT_LEFT = 0,
      /// \brief Right foot is support foot.
      SUPPORT_FOOT_RIGHT,

      /// \brief Enum maximum value, do not describe a valid phase.
      SUPPORT_FOOT_SIZE
    };

  explicit WalkMovement (const sot::DiscretizedTrajectory& leftFoot,
			 const sot::DiscretizedTrajectory& rightFoot,
			 const sot::DiscretizedTrajectory& com,
			 const sot::DiscretizedTrajectory& zmp,
			 const sot::MatrixHomogeneous& wMs);

  /// \brief Left foot trajectory.
  sot::DiscretizedTrajectory leftFoot;

  /// \brief Right foot trajectory.
  sot::DiscretizedTrajectory rightFoot;

  /// \brief Center of mass trajectory.
  sot::DiscretizedTrajectory com;

  /// \brief ZMP reference trajectory.
  sot::DiscretizedTrajectory zmp;

  /// \brief Trajectory frame position in the world frame.
  ///
  /// Trajectories are played from the current position but
  /// initial position may be different. To solve this issue,
  /// the world frame position w.r.t the start position
  /// is stored in this attribute.
  ///
  /// ${}^w X = {}^w M_s * {}^s X$
  sot::MatrixHomogeneous wMs;

  /// \brief Describes when the walk phase changes.
  ///
  /// Each element of the vector describes a new phase
  /// of the walk associated with its starting time.
  ///
  /// Exampple:
  ///  (0., SUPPORT_FOOT_DOUBLE)
  ///  (1., SUPPORT_FOOT_LEFT)
  ///  (3., SUPPORT_FOOT_RIGHT)
  ///  (5., SUPPORT_FOOT_DOUBLE)
  ///
  /// At t=0, double support then at t=1 switch to single support
  /// using left foot as support foot, then at t=3 switch to single
  /// support using right foot as support foot and finally starting at
  /// t=5. double support.
  std::vector<std::pair<double, SupportFoot> > supportFoot;
};

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

  void update (int t)
  {
     if (t <= t_)
       return;
    t_ = t;
    impl_update ();
  }

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

protected:
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
