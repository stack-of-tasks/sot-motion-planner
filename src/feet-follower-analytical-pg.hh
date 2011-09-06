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

#ifndef SOT_MOTION_PLANNER_FEET_FOLLOWER_ANALYTICAL_PG_HH
# define SOT_MOTION_PLANNER_FEET_FOLLOWER_ANALYTICAL_PG_HH
# include <string>

# include <boost/filesystem/path.hpp>
# include <boost/optional.hpp>

# include <dynamic-graph/command.h>

# include "discretized-trajectory.hh"
# include "feet-follower.hh"

# include "analytical-pg/newPGstepStudy.h"

class FeetFollowerAnalyticalPg;

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  class GenerateTrajectory : public Command
  {
  public:
    GenerateTrajectory (FeetFollowerAnalyticalPg& entity,
			const std::string& docstring);
    virtual Value doExecute ();
  };

  class PushStep : public Command
  {
  public:
    PushStep (FeetFollowerAnalyticalPg& entity,
	      const std::string& docstring);
    virtual Value doExecute ();
  };

  class ClearSteps : public Command
  {
  public:
    ClearSteps (FeetFollowerAnalyticalPg& entity,
		const std::string& docstring);
    virtual Value doExecute ();
  };
} // end of namespace command.

class FeetFollowerAnalyticalPg : public FeetFollower
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  static const double STEP;

  explicit FeetFollowerAnalyticalPg (const std::string& name);
  virtual ~FeetFollowerAnalyticalPg ();

  void generateTrajectory ();

  void pushStep (const ml::Vector& step);

  void clearSteps ()
  {
    steps_.clear ();
  }

  virtual boost::optional<const WalkMovement&> walkMovement () const
  {
    if (!trajectories_)
      return boost::optional<const WalkMovement&> ();
    return *trajectories_;
  }

  virtual boost::optional<int>
  trajectorySize () const
  {
    if (!trajectories_)
      return boost::optional<int> ();
    return trajectories_->leftFoot.trajectorySize ();
  }

  void
  setWaistFile (const std::string& waistFile)
  {
    waistFile_ = boost::filesystem::path (waistFile);
  }

private:
  virtual void impl_update ();
  void updateVelocities ();

  std::vector<ml::Vector> steps_;
  bool leftOrRightFootStable_;

  boost::optional<WalkMovement> trajectories_;
  unsigned index_;
  boost::filesystem::path waistFile_;
};

#endif //! SOT_MOTION_PLANNER_FEET_FOLLOWER_ANALYTICAL_PG_HH
