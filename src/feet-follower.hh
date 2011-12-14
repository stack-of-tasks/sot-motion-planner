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
# include <boost/shared_ptr.hpp>

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
computeAnklePositionInWorldFrame (double footX, double footY, double footZ,
				  double footYaw,
				  const sot::MatrixHomogeneous& fMa);

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

  class GetFinalLeftAnklePosition : public Command
  {
  public:
    GetFinalLeftAnklePosition (FeetFollower& entity,
			       const std::string& docstring);
    virtual Value doExecute();
  };

  class GetFinalRightAnklePosition : public Command
  {
  public:
    GetFinalRightAnklePosition (FeetFollower& entity,
			       const std::string& docstring);
    virtual Value doExecute();
  };

  class GetInitialLeftAnklePosition : public Command
  {
  public:
    GetInitialLeftAnklePosition (FeetFollower& entity,
			       const std::string& docstring);
    virtual Value doExecute();
  };

  class GetInitialRightAnklePosition : public Command
  {
  public:
    GetInitialRightAnklePosition (FeetFollower& entity,
			       const std::string& docstring);
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
  typedef std::vector<std::pair<double, SupportFoot> > supportFoot_t;
  typedef boost::shared_ptr<sot::DiscretizedTrajectory> trajectoryPtr_t;

  explicit WalkMovement (const trajectoryPtr_t& leftFoot,
			 const trajectoryPtr_t& rightFoot,
			 const trajectoryPtr_t& com,
			 const trajectoryPtr_t& zmp,
			 const trajectoryPtr_t& waistYaw,
			 const trajectoryPtr_t& waist,
			 const trajectoryPtr_t& gaze,
			 const trajectoryPtr_t& posture,
			 const sot::MatrixHomogeneous& wMs);

  /// \brief Left foot trajectory.
  trajectoryPtr_t leftFoot;

  /// \brief Right foot trajectory.
  trajectoryPtr_t rightFoot;

  /// \brief Center of mass trajectory.
  trajectoryPtr_t com;

  /// \brief ZMP reference trajectory.
  trajectoryPtr_t zmp;

  /// \brief Waist yaw reference trajectory.
  trajectoryPtr_t waistYaw;

  /// \brief Posture reference trajectory.
  trajectoryPtr_t posture;

  /// \brief Waist reference trajectory.
  trajectoryPtr_t waist;

  /// \brief Gaze reference trajectory.
  trajectoryPtr_t gaze;

  /// \brief Trajectory frame position in the world frame (wMw_traj).
  ///
  /// Trajectories are played from the current position but
  /// initial position may be different. To solve this issue,
  /// the world frame position w.r.t the start position
  /// is stored in this attribute.
  ///
  /// ${}^wM_{la} = {}^w M_{w_{traj}} * {}^{w_{traj}} M_{la}$
  sot::MatrixHomogeneous wMw_traj;

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
  supportFoot_t supportFoot;
};

class FeetFollower : public dg::Entity
{
public:
  typedef dg::SignalTimeDependent<ml::Vector, int> signalCoM_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalCoMVelocity_t;
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signalFoot_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalFootVelocity_t;

  explicit FeetFollower (const std::string& name);
  virtual ~FeetFollower ();

  void start ();
  void update (int t);
  ml::Vector& updateCoM (ml::Vector& res, int t);
  ml::Vector& updateZmp (ml::Vector& res, int t);
  sot::MatrixHomogeneous& updateWaistYaw (sot::MatrixHomogeneous& res, int t);
  sot::MatrixHomogeneous& updateWaist (sot::MatrixHomogeneous& res, int t);
  sot::MatrixHomogeneous& updateGaze (sot::MatrixHomogeneous& res, int t);
  sot::MatrixHomogeneous& updateLeftAnkle (sot::MatrixHomogeneous& res, int t);
  sot::MatrixHomogeneous& updateRightAnkle (sot::MatrixHomogeneous& res, int t);
  ml::Vector& updatePosture (ml::Vector& res, int t);

  ml::Vector& updateCoMVelocity (ml::Vector& res, int t);
  ml::Vector& updateWaistYawVelocity (ml::Vector& res, int t);
  ml::Vector& updateLeftAnkleVelocity (ml::Vector& res, int t);
  ml::Vector& updateRightAnkleVelocity (ml::Vector& res, int t);

  double getTime () const;

  /// \brief Returns the current time index on the trajectory.
  /// I.e. time - startTime (0 means the trajectory replay just began)
  double getTrajectoryTime () const;

  virtual boost::optional<const WalkMovement&> walkMovement () const = 0;

  virtual void impl_start ()
  {}

  signalCoM_t& comOut ()
  {
    return comOut_;
  }

  signalCoM_t& zmpOut ()
  {
    return zmpOut_;
  }

  signalFoot_t& leftAnkleOut ()
  {
    return leftAnkleOut_;
  }

  signalFoot_t& rightAnkleOut ()
  {
    return rightAnkleOut_;
  }

  signalCoMVelocity_t& comVelocityOut ()
  {
    return comVelocityOut_;
  }

  signalFootVelocity_t& waistYawVelocityOut ()
  {
    return waistYawVelocityOut_;
  }

  signalFootVelocity_t& leftAnkleVelocityOut ()
  {
    return leftAnkleVelocityOut_;
  }

  signalFootVelocity_t& rightAnkleVelocityOut ()
  {
    return rightAnkleVelocityOut_;
  }

  bool started () const
  {
    return started_;
  }

  double startTime () const;

  const sot::MatrixHomogeneous& leftFootToAnkle () const
  {
    return leftFootToAnkle_;
  }

  const sot::MatrixHomogeneous& rightFootToAnkle () const
  {
    return rightFootToAnkle_;
  }

  const maal::boost::Matrix& finalLeftAnklePosition () const
  {
    return finalLeftAnklePosition_;
  }

  const maal::boost::Matrix& finalRightAnklePosition () const
  {
    return finalRightAnklePosition_;
  }

  const maal::boost::Matrix& initialLeftAnklePosition () const
  {
    return initialLeftAnklePosition_;
  }

  const maal::boost::Matrix& initialRightAnklePosition () const
  {
    return initialRightAnklePosition_;
  }

  /// \brief Number of steps (positions) in the trajectory.
  virtual boost::optional<int> trajectorySize () const = 0;

protected:
  virtual void impl_update () = 0;

  int t_;
  ml::Vector com_;
  ml::Vector zmp_;
  sot::MatrixHomogeneous waistYaw_;
  sot::MatrixHomogeneous waist_;
  sot::MatrixHomogeneous gaze_;
  sot::MatrixHomogeneous leftAnkle_;
  sot::MatrixHomogeneous rightAnkle_;
  ml::Vector posture_;

  ml::Vector comVelocity_;
  ml::Vector waistYawVelocity_;
  ml::Vector leftAnkleVelocity_;
  ml::Vector rightAnkleVelocity_;

  double comZ_;
  sot::MatrixHomogeneous leftFootToAnkle_;
  sot::MatrixHomogeneous rightFootToAnkle_;
  sot::MatrixHomogeneous initialLeftAnklePosition_;
  sot::MatrixHomogeneous initialRightAnklePosition_;

  bool started_;
  double startTime_;

  signalCoM_t comOut_;
  signalCoM_t zmpOut_;
  signalFoot_t waistYawOut_;
  signalFoot_t waistOut_;
  signalFoot_t gazeOut_;
  signalFoot_t leftAnkleOut_;
  signalFoot_t rightAnkleOut_;
  signalCoM_t postureOut_;

  signalCoMVelocity_t comVelocityOut_;
  signalFootVelocity_t waistYawVelocityOut_;
  signalFootVelocity_t leftAnkleVelocityOut_;
  signalFootVelocity_t rightAnkleVelocityOut_;

  sot::MatrixHomogeneous finalLeftAnklePosition_;
  sot::MatrixHomogeneous finalRightAnklePosition_;

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
