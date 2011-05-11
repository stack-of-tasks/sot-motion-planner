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

#include <string>
#include <fstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>

#include <jrl/mal/boost.hh>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot-dynamic/dynamic.h>

#include "common.hh"
#include "discretized-trajectory.hh"
#include "feet-follower-from-file.hh"

const double FeetFollowerFromFile::STEP = 0.005;

WalkMovement loadFromFile(const fs::path& LeftFootPath,
			    const fs::path& RightFootPath,
			    const fs::path& ComPath,
			    const fs::path& ZmpPath,
			    const fs::path& WaistYawPath,
			    const double& step)
{
  return WalkMovement
    (sot::DiscretizedTrajectory::loadTrajectoryFromFile
     (LeftFootPath, step, "left-ankle"),
     sot::DiscretizedTrajectory::loadTrajectoryFromFile
     (RightFootPath, step, "right-ankle"),
     sot::DiscretizedTrajectory::loadTrajectoryFromFile
     (ComPath, step, "com"),
     sot::DiscretizedTrajectory::loadTrajectoryFromFile
     (ZmpPath, step, "zmp"),
     sot::DiscretizedTrajectory::loadTrajectoryFromFile
     (WaistYawPath, step, "waist-yaw"),
     sot::MatrixHomogeneous ());
}

using ::dynamicgraph::command::Setter;

FeetFollowerFromFile::FeetFollowerFromFile (const std::string& name)
  : FeetFollower (name),
    trajectories_ (),
    index_ (0)
{
  std::string docstring = "";
  addCommand ("readTrajectory", new Setter<FeetFollowerFromFile, std::string>
	      (*this, &FeetFollowerFromFile::readTrajectory, docstring));

}

void
FeetFollowerFromFile::readTrajectory (const std::string& dirname)
{
  namespace fs = boost::filesystem;

  fs::path trajectoryPath (dirname);
  fs::path trajectoryLeftFootPath = trajectoryPath / "left-ankle.dat";
  fs::path trajectoryRightFootPath = trajectoryPath / "right-ankle.dat";
  fs::path trajectoryComPath = trajectoryPath / "com.dat";
  fs::path trajectoryZmpPath = trajectoryPath / "zmp.dat";
  fs::path trajectoryWaistYawPath = trajectoryPath / "waist-yaw.dat";

  if (!fs::is_directory (trajectoryPath))
    {
      std::cerr << "invalid trajectory path" << std::endl;
      return;
    }

  if (!fs::exists (trajectoryLeftFootPath)
      || fs::is_directory (trajectoryLeftFootPath))
    {
      std::cerr << "invalid left-ankle trajectory file" << std::endl;
      return;
    }

  if (!fs::exists (trajectoryRightFootPath)
      || fs::is_directory (trajectoryRightFootPath))
    {
      std::cerr << "invalid right-ankle trajectory file" << std::endl;
      return;
    }

  if (!fs::exists (trajectoryComPath)
      || fs::is_directory (trajectoryComPath))
    {
      std::cerr << "invalid COM trajectory file" << std::endl;
      return;
    }

  if (!fs::exists (trajectoryZmpPath)
      || fs::is_directory (trajectoryZmpPath))
    {
      std::cerr << "invalid ZMP trajectory file" << std::endl;
      return;
    }

  if (!fs::exists (trajectoryWaistYawPath)
      || fs::is_directory (trajectoryWaistYawPath))
    {
      std::cerr << "invalid waist yaw trajectory file" << std::endl;
      return;
    }

  trajectories_ = loadFromFile (trajectoryLeftFootPath,
				trajectoryRightFootPath,
				trajectoryComPath,
				trajectoryZmpPath,
				trajectoryWaistYawPath,
				STEP);

  // Reset the movement.
  index_ = 0;

  const sot::Trajectory::vector_t& initialConfig =
    trajectories_->leftFoot (0.);

  // wMw_traj = wMa * aMw_traj = wMa * (w_trajMa)^{-1}
  //
  // wMa = ankle position in dynamic-graph frame (initialLeftAnklePosition_)
  // w_trajMa = ankle position in pattern generator frame (initialConfig)
  trajectories_->wMw_traj =
    initialLeftAnklePosition_
    * computeAnklePositionInWorldFrame (initialConfig[0], initialConfig[1],
					initialConfig[2], initialConfig[3],
					leftFootToAnkle_).inverse ();
}

FeetFollowerFromFile::~FeetFollowerFromFile ()
{}

void
FeetFollowerFromFile::impl_update ()
{
  using sot::Trajectory;
  using roboptim::Function;

  if (!trajectories_)
    return;

  const double t = index_ * STEP;

  if (t >= Function::getUpperBound (trajectories_->leftFoot.getRange ()))
    return;

  const Trajectory::vector_t& leftFoot = trajectories_->leftFoot (t);
  const Trajectory::vector_t& rightFoot = trajectories_->rightFoot (t);
  const Trajectory::vector_t& zmp = trajectories_->zmp (t);
  const Trajectory::vector_t& com = trajectories_->com (t);

  if (leftFoot.size () != 4 || rightFoot.size () != 4
      || com.size () != 2 || zmp.size () != 2)
    {
      std::cerr << "bad size" << std::endl;
      return;
    }

  // wMla = wMw_traj * w_trajMla
  leftAnkle_ =
    trajectories_->wMw_traj *
    computeAnklePositionInWorldFrame
    (leftFoot[0], leftFoot[1], leftFoot[2], leftFoot[3], leftFootToAnkle_);

  // wMra = wMw_traj * w_trajMra
  rightAnkle_ =
    trajectories_->wMw_traj *
    computeAnklePositionInWorldFrame
    (rightFoot[0], rightFoot[1], rightFoot[2], rightFoot[3],
     rightFootToAnkle_);

  ml::Vector comH (4);
  ml::Vector zmpH (4);

  for (unsigned i = 0; i < 2; ++i)
    comH (i) = com[i], zmpH (i) = zmp[i];

  comH (2) = comZ_;
  zmpH (2) = 0.;

  comH (3) = zmpH (3) = 1.;

  // {}^w com = wMw_traj * {}^{w_traj} com
  comH = trajectories_->wMw_traj * comH;
  // {}^w zmp = wMw_traj * {}^{w_traj} zmp
  zmpH = trajectories_->wMw_traj * zmpH;

  for (unsigned i = 0; i < 3; ++i)
    com_ (i) = comH (i), zmp_ (i) = zmpH (i);

  if (started_)
    ++index_;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (FeetFollowerFromFile,
				    "FeetFollowerFromFile");
