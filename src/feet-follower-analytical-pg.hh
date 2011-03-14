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

struct TrajectoriesAnalyticalPg
{
  TrajectoriesAnalyticalPg (const sot::DiscretizedTrajectory& leftFoot,
			    const sot::DiscretizedTrajectory& rightFoot,
			    const sot::DiscretizedTrajectory& com,
			    const sot::DiscretizedTrajectory& zmp,
			    const sot::MatrixHomogeneous& wMs);

  sot::DiscretizedTrajectory leftFoot;
  sot::DiscretizedTrajectory rightFoot;
  sot::DiscretizedTrajectory com;
  sot::DiscretizedTrajectory zmp;

  // Transform the pg global frame into the sot global frame.
  sot::MatrixHomogeneous wMs;
};


class FeetFollowerAnalyticalPg : public FeetFollower
{
public:
  static const std::string CLASS_NAME;

  static const double STEP;

  explicit FeetFollowerAnalyticalPg (const std::string& name);
  virtual ~FeetFollowerAnalyticalPg ();

  void generateTrajectory ();

  void pushStep (const ml::Vector& step);

  void clearSteps ()
  {
    steps_.clear ();
  }
private:
  virtual void impl_update ();

  std::vector<ml::Vector> steps_;
  bool leftOrRightFootStable_;

  boost::optional<TrajectoriesAnalyticalPg> trajectories_;
  unsigned index_;
};

#endif //! SOT_MOTION_PLANNER_FEET_FOLLOWER_ANALYTICAL_PG_HH
