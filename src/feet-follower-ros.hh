// Copyright 2011, Thomas Moulard, Olivier Stasse, JRL, CNRS/AIST.
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

#ifndef SOT_MOTION_PLANNER_FEET_FOLLOWER_ROS_HH
# define SOT_MOTION_PLANNER_FEET_FOLLOWER_ROS_HH
# include <string>
# include <boost/optional.hpp>
# include <boost/filesystem.hpp>

# include <sot/core/matrix-homogeneous.hh>
# include <walk_interfaces/pattern-generator.hh>

# include "discretized-trajectory.hh"
# include "feet-follower.hh"

namespace walk
{
  struct ReaderPatternGenerator2d;
  template <>
  struct PatternGeneratorTraits<ReaderPatternGenerator2d>
  {
    /// \brief Footprint definition.
    typedef walk::StampedFootprint2d Footprint;

    /// \brief Trajectory in \f$SO(3)\f$.
    typedef DiscretizedTrajectory<StampedPosition3d> Trajectory3d;
    /// \brief Trajectory in \f$SO(2)\f$.
    typedef DiscretizedTrajectory<StampedPosition2d> Trajectory2d;
    /// \brief Trajectory in \f$R^2\f$.
    typedef DiscretizedTrajectory<StampedVector2d> TrajectoryV2d;
    /// \brief Trajectory in \f$R^3\f$.
    typedef DiscretizedTrajectory<StampedVector3d> TrajectoryV3d;
    /// \brief Trajectory in \f$R^n\f$.
    typedef DiscretizedTrajectory<StampedVectorNd> TrajectoryNd;
  };

  class ReaderPatternGenerator2d
    : public walk::PatternGenerator<ReaderPatternGenerator2d>
  {
  public:
    virtual void computeTrajectories()
    {}
  };
} // end of namespace walk.

class FeetFollowerRos : public FeetFollower
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  static const double STEP;

  explicit FeetFollowerRos (const std::string& name);
  virtual ~FeetFollowerRos ();

  virtual boost::optional<const WalkMovement&> walkMovement () const
  {
    if (!trajectories_)
      return boost::optional<const WalkMovement&> ();
    return *trajectories_;
  }

  virtual boost::optional<int>
  trajectorySize () const
  {
    if (!trajectories_ || !trajectories_->leftFoot)
      return boost::optional<int> ();
    return trajectories_->leftFoot->trajectorySize ();
  }

  void parseTrajectory (const std::string& trajectory);

  bool canStart () const
  {
    return ready_;
  }

private:
  virtual void impl_update ();
  virtual void impl_start ();
  void updateVelocities ();

private:
  volatile bool ready_;
  boost::optional<WalkMovement> trajectories_;
  boost::optional<walk::ReaderPatternGenerator2d::footprints_t> footprints_;
  unsigned index_;
};

#endif //! SOT_MOTION_PLANNER_FEET_FOLLOWER_ROS_HH
