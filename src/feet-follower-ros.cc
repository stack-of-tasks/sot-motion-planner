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

#include <sstream>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "common.hh"
#include "discretized-trajectory.hh"
#include "feet-follower-ros.hh"
#include "discretized-trajectory.hh"

#include <walk_interfaces/pattern-generator.hh>
#include <walk_interfaces/yaml.hh>

const double FeetFollowerRos::STEP = 0.005;

namespace command
{
  using ::dynamicgraph::command::Command;

  class RetrieveTrajectory : public Command
  {
  public:
    RetrieveTrajectory (FeetFollowerRos& entity,
			const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::STRING),
	 docstring)
    {
    }

    virtual Value doExecute ()
    {
      FeetFollowerRos& feetFollowerRos =
	static_cast<FeetFollowerRos&> (owner ());
      const std::vector<Value>& values = getParameterValues();
      const std::string& trajectory = values[0].value();
      feetFollowerRos.parseTrajectory (trajectory);
    }
  };
} // end of namespace command.

FeetFollowerRos::FeetFollowerRos (const std::string& name)
  : FeetFollower (name),
    trajectories_ (),
    index_ (0)
{
  std::string docstring = "";
}

FeetFollowerRos::~FeetFollowerRos ()
{}

void
FeetFollowerRos::impl_update ()
{
  using sot::Trajectory;
  using roboptim::Function;

  if (!trajectories_)
    return;

  const double t = index_ * STEP;

  if (t >= Function::getUpperBound (trajectories_->leftFoot->getRange ()))
    return;

  const Trajectory::vector_t& leftFoot = (*trajectories_->leftFoot) (t);
  const Trajectory::vector_t& rightFoot = (*trajectories_->rightFoot) (t);
  const Trajectory::vector_t& zmp = (*trajectories_->zmp) (t);
  const Trajectory::vector_t& com = (*trajectories_->com) (t);

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

class ReaderPatternGenerator2d : public walk::PatternGenerator2d
{
public:
  virtual void computeTrajectories()
  {}
};

// FIXME: here we make the assumption that the trajectory
// contains data points which are discretized every 5ms.
// This is not necessary the case!
void
FeetFollowerRos::parseTrajectory (const std::string& trajectory)
{
  try
    {
      typedef roboptim::Function::vector_t vector_t;
      typedef roboptim::Function::discreteInterval_t interval_t;
      typedef boost::shared_ptr<sot::DiscretizedTrajectory> trajectoryPtr_t;
      std::stringstream ss;
      ss << trajectory;
      walk::YamlReader<ReaderPatternGenerator2d> reader (ss);

      walk::TimeDuration td = reader.leftFootTrajectory ().computeLength ();
      interval_t range
	(0.,
	 td.total_microseconds () * 1e-6,
	 STEP);
      std::vector<vector_t> leftFootData;
      std::vector<vector_t> rightFootData;
      std::vector<vector_t> comData;
      std::vector<vector_t> zmpData;
      std::vector<vector_t> waistYawData;
      std::vector<vector_t> waistData;
      std::vector<vector_t> gazeData;

      vector_t leftFootElt (4);
      vector_t rightFootElt (4);
      vector_t comElt (3);
      vector_t zmpElt (3);
      for (unsigned i = 0;
	   i < reader.leftFootTrajectory ().data ().size (); ++i)
	{
	  leftFootElt (0) =
	    reader.leftFootTrajectory ().data ()[i].position (0, 3);
	  leftFootElt (1) =
	    reader.leftFootTrajectory ().data ()[i].position (1, 3);
	  leftFootElt (2) =
	    reader.leftFootTrajectory ().data ()[i].position (2, 3);
	  leftFootElt (3) =
	    std::atan2
	    (reader.leftFootTrajectory ().data ()[i].position (1, 0),
	     reader.leftFootTrajectory ().data ()[i].position (0, 0));

	  rightFootElt (0) =
	    reader.rightFootTrajectory ().data ()[i].position (0, 3);
	  rightFootElt (1) =
	    reader.rightFootTrajectory ().data ()[i].position (1, 3);
	  rightFootElt (2) =
	    reader.rightFootTrajectory ().data ()[i].position (2, 3);
	  rightFootElt (3) =
	    std::atan2
	    (reader.rightFootTrajectory ().data ()[i].position (1, 0),
	     reader.rightFootTrajectory ().data ()[i].position (0, 0));

	  comElt (0) =
	    reader.centerOfMassTrajectory ().data ()[i].position[0];
	  comElt (1) =
	    reader.centerOfMassTrajectory ().data ()[i].position[1];
	  zmpElt (0) =
	    reader.zmpTrajectory ().data ()[i].position[0];
	  zmpElt (1) =
	    reader.zmpTrajectory ().data ()[i].position[1];

	  leftFootData.push_back (leftFootElt);
	  rightFootData.push_back (rightFootElt);
	  comData.push_back (comElt);
	  zmpData.push_back (zmpElt);
	}

      trajectoryPtr_t leftFoot = boost::make_shared<sot::DiscretizedTrajectory>
	(range, leftFootData, "left foot from ros");
      trajectoryPtr_t rightFoot = boost::make_shared<sot::DiscretizedTrajectory>
	(range, rightFootData, "right foot from ros");
      trajectoryPtr_t com = boost::make_shared<sot::DiscretizedTrajectory>
	(range, comData, "com from ros");
      trajectoryPtr_t zmp = boost::make_shared<sot::DiscretizedTrajectory>
	(range, zmpData, "zmp from ros");
      trajectoryPtr_t waistYaw = boost::make_shared<sot::DiscretizedTrajectory>
	(range, waistYawData, "waist yaw from ros");
      trajectoryPtr_t waist = boost::make_shared<sot::DiscretizedTrajectory>
	(range, waistYawData, "waist trajectory from ros");
      trajectoryPtr_t gaze = boost::make_shared<sot::DiscretizedTrajectory>
	(range, waistYawData, "gaze trajectory from ros");

      sot::MatrixHomogeneous wMs;

      trajectories_ = WalkMovement (leftFoot, rightFoot, com, zmp,
				    waistYaw, waist, gaze, wMs);
    }
  catch (std::exception& e)
    {
      std::cerr << e.what () << std::endl;
    }
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (FeetFollowerRos, "FeetFollowerRos");
