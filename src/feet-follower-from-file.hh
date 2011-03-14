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

#ifndef SOT_MOTION_PLANNER_FEET_FOLLOWER_FROM_FILE_HH
# define SOT_MOTION_PLANNER_FEET_FOLLOWER_FROM_FILE_HH
# include <string>
# include <boost/optional.hpp>
# include <boost/filesystem.hpp>

# include <sot/core/matrix-homogeneous.hh>

# include "discretized-trajectory.hh"
# include "feet-follower.hh"

namespace fs = boost::filesystem;

struct Trajectories
{
  Trajectories (const fs::path& LeftFootPath,
		const fs::path& RightFootPath,
		const fs::path& ComPath,
		const fs::path& ZmpPath,
		const double& step);
  sot::DiscretizedTrajectory leftFoot;
  sot::DiscretizedTrajectory rightFoot;
  sot::DiscretizedTrajectory com;
  sot::DiscretizedTrajectory zmp;

  // Transform the pg global frame into the sot global frame.
  sot::MatrixHomogeneous wMs;
};

class FeetFollowerFromFile : public FeetFollower
{
public:
  static const std::string CLASS_NAME;

  static const double STEP;

  explicit FeetFollowerFromFile (const std::string& name);
  virtual ~FeetFollowerFromFile ();

  void readTrajectory (const std::string& dirname);

private:
  virtual void impl_update ();

private:
  boost::optional<Trajectories> trajectories_;
  unsigned index_;
};

#endif //! SOT_MOTION_PLANNER_FEET_FOLLOWER_FROM_FILE_HH
