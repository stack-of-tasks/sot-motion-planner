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

#include <cassert>

#include <ros/ros.h>

#include <sstream>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

#include "common.hh"
#include "discretized-trajectory.hh"
#include "feet-follower-ros.hh"
#include "discretized-trajectory.hh"

#include <dynamic_graph_bridge/ros_init.hh>
#include <walk_interfaces/pattern-generator.hh>
#include <walk_interfaces/binary.hh>

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
      const std::string rosParameter = values[0].value();

      boost::thread
	(boost::bind
	 (&FeetFollowerRos::parseTrajectory,
	  boost::ref (feetFollowerRos), rosParameter));
      return Value ();
    }
  };

  class CanStart : public Command
  {
  public:
    CanStart (FeetFollowerRos& entity,
			const std::string& docstring)
      : Command (entity, std::vector<Value::Type> (), docstring)
    {}

    virtual Value doExecute ()
    {
      FeetFollowerRos& feetFollowerRos =
	static_cast<FeetFollowerRos&> (owner ());

      return Value (feetFollowerRos.canStart ());
    }
  };
} // end of namespace command.

FeetFollowerRos::FeetFollowerRos (const std::string& name)
  : FeetFollower (name),
    ready_ (false),
    trajectories_ (),
    footprints_ (),
    index_ (0)
{
  std::string docstring = "";
  addCommand ("parseTrajectory", new command::RetrieveTrajectory
	      (*this, docstring));
  addCommand ("canStart", new command::CanStart
	      (*this, docstring));
}

FeetFollowerRos::~FeetFollowerRos ()
{}

void
FeetFollowerRos::impl_start ()
{
  if (!ready_)
    throw std::runtime_error ("failed to call start");
  std::cout << "FeetFollower ROS started" << std::endl;
}

void
FeetFollowerRos::updateVelocities ()
{
  using sot::Trajectory;
  using roboptim::Function;

  const double t = (index_) * STEP;
  const double tnext = (index_ + 1) * STEP;

  if (t >= Function::getUpperBound (trajectories_->leftFoot->getRange ()) ||
      tnext >= Function::getUpperBound (trajectories_->leftFoot->getRange ()))
    {
      comVelocity_.setZero ();
      waistYawVelocity_.setZero ();
      leftAnkleVelocity_.setZero ();
      rightAnkleVelocity_.setZero ();
      leftWristVelocity_.setZero ();
      rightWristVelocity_.setZero ();
      return;
    }

  const Trajectory::vector_t& leftFoot = (*trajectories_->leftFoot) (t);
  const Trajectory::vector_t& rightFoot = (*trajectories_->rightFoot) (t);
  const Trajectory::vector_t& com = (*trajectories_->com) (t);
  const Trajectory::vector_t& waistYaw = (*trajectories_->waistYaw) (t);

  const Trajectory::vector_t& leftFootNext = (*trajectories_->leftFoot) (tnext);
  const Trajectory::vector_t& rightFootNext =
    (*trajectories_->rightFoot) (tnext);
  const Trajectory::vector_t& comNext = (*trajectories_->com) (tnext);
  const Trajectory::vector_t& waistYawNext = (*trajectories_->waistYaw) (tnext);

  comVelocity_.accessToMotherLib () = (comNext - com) / STEP;

  waistYawVelocity_ (0) = 0.;
  waistYawVelocity_ (1) = 0.;
  waistYawVelocity_ (2) = waistYawNext[0] - waistYaw[0];

  //FIXME: foot <-> ankle
  leftAnkleVelocity_.setZero ();
  leftAnkleVelocity_ (0) = (leftFootNext[0] - leftFoot[0]) / STEP;
  leftAnkleVelocity_ (1) = (leftFootNext[1] - leftFoot[1]) / STEP;
  leftAnkleVelocity_ (5) = (leftFootNext[2] - leftFoot[2]) / STEP;

  rightAnkleVelocity_.setZero ();
  rightAnkleVelocity_ (0) = (rightFootNext[0] - rightFoot[0]) / STEP;
  rightAnkleVelocity_ (1) = (rightFootNext[1] - rightFoot[1]) / STEP;
  rightAnkleVelocity_ (5) = (rightFootNext[2] - rightFoot[2]) / STEP;

  // for now null speed assumed.
  leftWristVelocity_.setZero ();
  rightWristVelocity_.setZero ();
}

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
  const Trajectory::vector_t& leftWrist = (*trajectories_->leftWrist) (t);
  const Trajectory::vector_t& rightWrist = (*trajectories_->rightWrist) (t);
  const Trajectory::vector_t& zmp = (*trajectories_->zmp) (t);
  const Trajectory::vector_t& com = (*trajectories_->com) (t);
  const Trajectory::vector_t& waistYaw = (*trajectories_->waistYaw) (t);
  const Trajectory::vector_t& posture =  (*trajectories_->posture) (t);

  if (leftFoot.size () != 4 || rightFoot.size () != 4
      || com.size () != 3 || zmp.size () != 2)
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

  // wrists
  for (unsigned i = 0; i < 4; ++i)
    for (unsigned j = 0; j < 4; ++j)
      {
	leftWrist_ (i, j) = leftWrist[i * 4 + j];
	leftWrist_ (i, j) = rightWrist[i * 4 + j];
      }

  ml::Vector comH (4);
  ml::Vector zmpH (4);

  for (unsigned i = 0; i < 2; ++i)
    comH (i) = com[i], zmpH (i) = zmp[i];

  comH (2) = com[2];
  zmpH (2) = 0.;

  comH (3) = zmpH (3) = 1.;

  // {}^w com = wMw_traj * {}^{w_traj} com
  comH = trajectories_->wMw_traj * comH;
  // {}^w zmp = wMw_traj * {}^{w_traj} zmp
  zmpH = trajectories_->wMw_traj * zmpH;

  for (unsigned i = 0; i < 3; ++i)
    com_ (i) = comH (i), zmp_ (i) = zmpH (i);

  jrlMathTools::Angle theta (waistYaw[0]);
  ml::Vector xytheta (3);
  xytheta (0) = xytheta (1) = 0.;
  xytheta (2) = theta.value ();

  waistYaw_ =
    trajectories_->wMw_traj * XYThetaToMatrixHomogeneous (xytheta);
  waistYaw_ (0, 3) = 0.;
  waistYaw_ (1, 3) = 0.;
  waistYaw_ (2, 3) = 0.;

  if (posture_.size () != posture.size ())
    posture_.resize (posture.size ());
  for (unsigned i = 0; i < posture.size (); ++i)
    posture_ (i) = posture[i];

  updateVelocities ();

  if (started_)
    ++index_;
}

// FIXME: here we make the assumption that the trajectory
// contains data points which are discretized every 5ms.
// This is not necessary the case!
void
FeetFollowerRos::parseTrajectory (const std::string& rosParameter)
{
  ready_ = false;
  try
    {
      ros::NodeHandle& nh = dynamicgraph::rosInit(false);
      XmlRpc::XmlRpcValue trajectory;
      if (!nh.getParam (rosParameter, trajectory))
	throw std::runtime_error ("failed to retrieve trajectory");

      std::vector<char>& rawTrajectory = trajectory;
      typedef roboptim::Function::vector_t vector_t;
      typedef roboptim::Function::discreteInterval_t interval_t;
      typedef boost::shared_ptr<sot::DiscretizedTrajectory> trajectoryPtr_t;
      std::stringstream ss(std::stringstream::in
			   | std::stringstream::out
			   | std::stringstream::binary);
      for (unsigned i = 0; i < rawTrajectory.size(); ++i)
	ss << rawTrajectory[i];
      walk::BinaryReader<walk::ReaderPatternGenerator2d> reader (ss);
      // walk::BinaryReader<walk::ReaderPatternGenerator2d> reader
      // 	("/tmp/walk.bin");
      walk::TimeDuration td = reader.leftFootTrajectory ().computeLength ();
      interval_t range
	(0.,
	 td.total_microseconds () * 1e-6,
	 STEP);

      std::vector<vector_t> leftFootData;
      std::vector<vector_t> rightFootData;
      std::vector<vector_t> leftWristData;
      std::vector<vector_t> rightWristData;
      std::vector<vector_t> comData;
      std::vector<vector_t> zmpData;
      std::vector<vector_t> waistYawData;
      std::vector<vector_t> waistData;
      std::vector<vector_t> gazeData;
      std::vector<vector_t> postureData;

      vector_t leftFootElt (4);
      vector_t rightFootElt (4);
      vector_t leftWristElt (4 * 4);
      vector_t rightWristElt (4 * 4);
      vector_t comElt (3);
      vector_t zmpElt (2);
      vector_t waistYawElt (1);
      vector_t postureElt (36);

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

	  for (unsigned tmp = 0; tmp < 4; ++tmp)
	    for (unsigned tmp2 = 0; tmp2 < 4; ++tmp2)
	      leftWristElt (tmp * 4 + tmp2) =
		reader.leftFootTrajectory ().data ()[i].position (tmp, tmp2);

	  comElt (0) =
	    reader.centerOfMassTrajectory ().data ()[i].position[0];
	  comElt (1) =
	    reader.centerOfMassTrajectory ().data ()[i].position[1];
	  comElt (2) =
	    reader.centerOfMassTrajectory ().data ()[i].position[2];
	  zmpElt (0) =
	    reader.zmpTrajectory ().data ()[i].position[0];
	  zmpElt (1) =
	    reader.zmpTrajectory ().data ()[i].position[1];

	  //FIXME: not generic enough.
	  waistYawElt (0) = reader.postureTrajectory ().data ()[i].position[0];

	  // Yaw only.
	  if (reader.postureTrajectory ().data ()[i].position.rows () == 1)
	    {
	      // Set to half-sitting.
	      static const double halfSitting[36] = {
		// Free flyer
		0., 0., 0.648702, 0., 0. , 0.,

		// Legs
		0., 0., -0.453786, 0.872665, -0.418879, 0.,
		0., 0., -0.453786, 0.872665, -0.418879, 0.,

		// Chest and head
		0., 0., 0., 0.,

		// Arms
		0.261799, -0.17453, 0., -0.523599, 0., 0., 0.1,
		0.261799, 0.17453,  0., -0.523599, 0., 0., 0.1
	      };

	      for (unsigned tmp = 0; tmp < 6 + 30; ++tmp)
		postureElt (tmp) = halfSitting[tmp];

	      // Override yaw using data.
	      postureElt (5) =
		reader.postureTrajectory ().data ()[i].position[0];
	    }
	  // Full body.
	  else if
	    (reader.postureTrajectory ().data ()[i].position.rows () == 36)
	    {
	      for (unsigned tmp = 0; tmp < 6 + 12; ++tmp)
		postureElt (tmp) = 0.;

	      for (unsigned j = 0; j < postureElt.size () - 6 - 12; ++j)
		postureElt (j + 6 + 12) =
		  reader.postureTrajectory ().data ()[i].position[j + 1];
	    }

	  leftFootData.push_back (leftFootElt);
	  rightFootData.push_back (rightFootElt);
	  leftWristData.push_back (leftWristElt);
	  rightWristData.push_back (rightWristElt);
	  comData.push_back (comElt);
	  zmpData.push_back (zmpElt);
	  waistYawData.push_back (waistYawElt);
	  postureData.push_back (postureElt);
	}

      trajectoryPtr_t leftFoot = boost::make_shared<sot::DiscretizedTrajectory>
	(range, leftFootData, "left foot from ros");
      trajectoryPtr_t rightFoot = boost::make_shared<sot::DiscretizedTrajectory>
	(range, rightFootData, "right foot from ros");
      trajectoryPtr_t leftWrist = boost::make_shared<sot::DiscretizedTrajectory>
	(range, leftWristData, "left wrist from ros");
      trajectoryPtr_t rightWrist = boost::make_shared<sot::DiscretizedTrajectory>
	(range, rightWristData, "right wrist from ros");
      trajectoryPtr_t com = boost::make_shared<sot::DiscretizedTrajectory>
	(range, comData, "com from ros");
      trajectoryPtr_t zmp = boost::make_shared<sot::DiscretizedTrajectory>
	(range, zmpData, "zmp from ros");
      trajectoryPtr_t waistYaw = boost::make_shared<sot::DiscretizedTrajectory>
	(range, waistYawData, "waist yaw from ros");
      trajectoryPtr_t posture = boost::make_shared<sot::DiscretizedTrajectory>
	(range, postureData, "posture from ros");

      // Empty trajectories.
      waistData.resize (zmpData.size (), vector_t (16));
      trajectoryPtr_t waist = boost::make_shared<sot::DiscretizedTrajectory>
	(range, waistData, "waist trajectory from ros");
      gazeData.resize (zmpData.size (), vector_t (16));
      trajectoryPtr_t gaze = boost::make_shared<sot::DiscretizedTrajectory>
	(range, gazeData, "gaze trajectory from ros");

      // wMw_traj = wMa * aMw_traj = wMa * (w_trajMa)^{-1}
      //
      // wMa = ankle position in dynamic-graph frame (initialLeftAnklePosition_)
      // w_trajMa = ankle position in pattern generator frame (initialConfig)
      sot::MatrixHomogeneous wMw_traj =
	initialLeftAnklePosition_
	* computeAnklePositionInWorldFrame (leftFootData[0][0], leftFootData[0][1],
					    leftFootData[0][2], leftFootData[0][3],
					    leftFootToAnkle_).inverse ();

      trajectories_ = WalkMovement (leftFoot, rightFoot,
				    leftWrist, rightWrist, com, zmp,
				    waistYaw, waist, gaze, posture, wMw_traj);
      footprints_ = reader.footprints ();


      // Compute support feet.
      double t = 0.;
      WalkMovement::SupportFoot supportFoot =
	WalkMovement::SUPPORT_FOOT_DOUBLE;

      if (reader.startWithLeftFoot())
	supportFoot = WalkMovement::SUPPORT_FOOT_RIGHT;
      else
	supportFoot = WalkMovement::SUPPORT_FOOT_LEFT;

      using namespace boost::gregorian;
      for (unsigned i = 1; i < reader.footprints ().size (); ++i)
	{
	  const walk::ReaderPatternGenerator2d::footprint_t& current =
	    reader.footprints ()[i];
	  const walk::ReaderPatternGenerator2d::footprint_t& previous =
	    reader.footprints ()[i - 1];

	  t = current.beginTime.time_of_day ().total_nanoseconds () / 1e9;
	  trajectories_->supportFoot.push_back
	    (std::make_pair (t, WalkMovement::SUPPORT_FOOT_DOUBLE));

	  // Then a feet is lifted.
	  t = (previous.beginTime.time_of_day ()
	       + previous.duration).total_nanoseconds () / 1e9;

	  trajectories_->supportFoot.push_back
	    (std::make_pair (t, supportFoot));

	  if (supportFoot == WalkMovement::SUPPORT_FOOT_LEFT)
	    supportFoot = WalkMovement::SUPPORT_FOOT_RIGHT;
	  else if (supportFoot == WalkMovement::SUPPORT_FOOT_RIGHT)
	    supportFoot = WalkMovement::SUPPORT_FOOT_LEFT;
	  else
	    assert (0 && "should never happen");
	}
    }
  catch (std::exception& e)
    {
      std::cerr << e.what () << std::endl;
    }
  ready_ = true;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (FeetFollowerRos, "FeetFollowerRos");
