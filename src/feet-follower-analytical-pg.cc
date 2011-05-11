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

#include "discretized-trajectory.hh"

#include "feet-follower-analytical-pg.hh"

const double FeetFollowerAnalyticalPg::STEP = 0.005;


FeetFollowerAnalyticalPg::FeetFollowerAnalyticalPg (const std::string& name)
  : FeetFollower (name),
    steps_ (),
    leftOrRightFootStable_ (true),
    trajectories_ (),
    index_ (0)
{
  std::string docstring = "";

  addCommand ("generateTrajectory",
	      new command::GenerateTrajectory (*this, docstring));
  addCommand ("pushStep",
	      new command::PushStep (*this, docstring));
  addCommand ("clearSteps",
	      new command::ClearSteps (*this, docstring));
}

FeetFollowerAnalyticalPg::~FeetFollowerAnalyticalPg ()
{}

void
FeetFollowerAnalyticalPg::impl_update ()
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
  const Trajectory::vector_t& waistYaw = trajectories_->waistYaw (t);

  if (leftFoot.size () != 4 || rightFoot.size () != 4
      || com.size () != 3 || zmp.size () != 3)
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

  for (unsigned i = 0; i < 3; ++i)
    comH (i) = com[i], zmpH (i) = zmp[i];
  comH (3) = zmpH (3) = 1.;

  // {}^w com = wMw_traj * {}^{w_traj} com
  comH = trajectories_->wMw_traj * comH;
  // {}^w zmp = wMw_traj * {}^{w_traj} zmp
  zmpH = trajectories_->wMw_traj * zmpH;

  for (unsigned i = 0; i < 3; ++i)
    com_ (i) = comH (i), zmp_ (i) = zmpH (i);

  jrlMathTools::Angle theta (waistYaw[0]);
  waistYaw_ (0) = theta.value ();

  if (started_)
    ++index_;
}

namespace
{
  void logStepFeatures(const std::vector<double>& steps,
		       const StepFeatures& stepFeatures,
		       const ml::Matrix& wMs)
  {
    std::ofstream log ("/tmp/pg.dat");

    log << "# it lf_x lf_y lf_z rf_x rf_y rf_z "
	<< "com_x com_y com_z zmp_x zmp_y w_x w_y w_z"
	<< std::endl;

    for (unsigned i = 0; i < stepFeatures.size; ++i)
    {
      boost::format fmt
	("%d "
	 "%f %f %f "
	 "%f %f %f "
	 "%f %f %f "
	 "%f %f");
      fmt
	% i

	% stepFeatures.leftfootXtraj[i]
	% stepFeatures.leftfootYtraj[i]
	% stepFeatures.leftfootHeight[i]

	% stepFeatures.rightfootXtraj[i]
	% stepFeatures.rightfootYtraj[i]
	% stepFeatures.rightfootHeight[i]

	% stepFeatures.comTrajX[i]
	% stepFeatures.comTrajY[i]
	% stepFeatures.zc

	% stepFeatures.zmpTrajX[i]
	% stepFeatures.zmpTrajY[i];

      log << fmt.str () << std::endl;
    }

    std::ofstream wMsLog ("/tmp/wms.dat");
    wMsLog << wMs << std::endl;

    std::ofstream logSteps ("/tmp/pg_steps.dat");

    for (unsigned i = 0; i < steps.size (); ++i)
      logSteps << steps[i] << std::endl;
  }
} // end of anonymous namespace.

void
FeetFollowerAnalyticalPg::generateTrajectory ()
{
  typedef sot::Trajectory::vector_t vector_t;
  typedef sot::Trajectory::discreteInterval_t discreteInterval_t;

  CnewPGstepStudy pg;

  static const double g = 9.81;
  static const double timeBeforeZmpShift = 0.95;
  static const double timeAfterZmpShift = 1.05;
  static const double halfStepLength = 2.;

  StepFeatures stepFeatures;

  leftOrRightFootStable_ = true;

  sot::MatrixHomogeneous initialLeftFeet =
    initialLeftAnklePosition_ * leftFootToAnkle_.inverse ();

  sot::MatrixHomogeneous initialRightFeet =
    initialRightAnklePosition_ * rightFootToAnkle_.inverse ();

  ml::Vector initialStep (6);


  // The first six parameters are feet position w.r.t center of mass
  // position.

  // As leftPosition_x = -rightPosition_x, leftPosition_y = -rightPosition_y,
  // we center the movement in (0, 0) and shift it back using wMs.
  initialStep (0) =
    std::fabs (initialLeftFeet (0, 3) - initialRightFeet (0, 3)) / 2.;
  initialStep (1) =
    std::fabs (initialLeftFeet (1, 3) - initialRightFeet (1, 3)) / 2.;
  initialStep (2) = 0.;

  initialStep (3) = -initialStep (0);
  initialStep (4) = -initialStep (1);
  initialStep (5) = atan2(initialRightFeet (1,0), initialRightFeet (0,0));

  std::vector<double> steps;
  for (unsigned i = 0; i < 6; ++i)
    steps.push_back (initialStep (i));

  for (unsigned i = 0; i < steps_.size (); ++i)
    {
      assert (steps_[i].size () == 7);

      // Make sure that slides coefficients are valid.
      if (steps_[i] (0) < -1.52 || steps_[i] (0) > 0.)
	throw std::runtime_error ("invalid first slide");
      if (steps_[i] (3) < -0.76 || steps_[i] (3) > 0.)
	throw std::runtime_error ("invalid second slide");

      // Convert from radian to degrees.
      steps_[i] (6) *= 180. / M_PI;

      for (unsigned j = 0; j < steps_[i].size (); ++j)
	steps.push_back (steps_[i] (j));
    }

  pg.produceSeqSlidedHalfStepFeatures
    (stepFeatures, STEP, comZ_, g,
     timeBeforeZmpShift, timeAfterZmpShift, halfStepLength, steps,
     leftOrRightFootStable_ ? 'L' : 'R');

  std::vector<vector_t> leftFootData;
  std::vector<vector_t> rightFootData;
  std::vector<vector_t> comData;
  std::vector<vector_t> zmpData;
  std::vector<vector_t> waistYawData;

  for (unsigned i = 0; i < stepFeatures.size; ++i)
    {
      vector_t leftFoot (4);
      vector_t rightFoot (4);
      vector_t com (3);
      vector_t zmp (3);
      vector_t waistYaw (1);

      // Convert from degrees into radians.
      jrlMathTools::Angle waistOrient
	(stepFeatures.waistOrient[i] * M_PI / 180.);
      jrlMathTools::Angle leftFootOrient
	(stepFeatures.leftfootOrient[i] * M_PI / 180.);
      jrlMathTools::Angle rightFootOrient
	(stepFeatures.rightfootOrient[i] * M_PI / 180.);

      leftFoot[0] = stepFeatures.leftfootXtraj[i];
      leftFoot[1] = stepFeatures.leftfootYtraj[i];
      leftFoot[2] = stepFeatures.leftfootHeight[i];
      leftFoot[3] = leftFootOrient.value ();

      rightFoot[0] = stepFeatures.rightfootXtraj[i];
      rightFoot[1] = stepFeatures.rightfootYtraj[i];
      rightFoot[2] = stepFeatures.rightfootHeight[i];
      rightFoot[3] = rightFootOrient.value ();

      com[0] = stepFeatures.comTrajX[i];
      com[1] = stepFeatures.comTrajY[i];
      com[2] = comZ_;

      zmp[0] = stepFeatures.zmpTrajX[i];
      zmp[1] = stepFeatures.zmpTrajY[i];
      zmp[2] = 0.;

      waistYaw[0] = waistOrient.value ();

      leftFootData.push_back (leftFoot);
      rightFootData.push_back (rightFoot);
      comData.push_back (com);
      zmpData.push_back (zmp);
      waistYawData.push_back (waistYaw);
    }

  // Reset the movement.
  index_ = 0;

  const sot::Trajectory::vector_t& initialConfig = leftFootData[0];

  // wMw_traj = wMa * aMw_traj = wMa * (w_trajMa)^{-1}
  //
  // wMa = ankle position in dynamic-graph frame (initialLeftAnklePosition_)
  // w_trajMa = ankle position in pattern generator frame (initialConfig)
  sot::MatrixHomogeneous wMw_traj =
    initialLeftAnklePosition_
    * computeAnklePositionInWorldFrame (initialConfig[0], initialConfig[1],
					initialConfig[2], initialConfig[3],
					leftFootToAnkle_).inverse ();

  logStepFeatures(steps, stepFeatures, wMw_traj);

  discreteInterval_t range (0., stepFeatures.size * STEP, STEP);

  trajectories_ = WalkMovement
    (sot::DiscretizedTrajectory (range, leftFootData, "left-foot"),
     sot::DiscretizedTrajectory (range, rightFootData, "right-foot"),
     sot::DiscretizedTrajectory (range, comData, "com"),
     sot::DiscretizedTrajectory (range, zmpData, "zmp"),
     sot::DiscretizedTrajectory (range, waistYawData, "waist-yaw"),
     wMw_traj);


  //FIXME: for now compute walk phases by looking at foot height.
  // It would probably better to recompute from timing parameters.
  trajectories_->supportFoot.push_back
    (std::make_pair (0., WalkMovement::SUPPORT_FOOT_DOUBLE));
  WalkMovement::SupportFoot oldPhase = WalkMovement::SUPPORT_FOOT_DOUBLE;
  for (unsigned i = 0; i < stepFeatures.size; ++i)
    {
      WalkMovement::SupportFoot phase = WalkMovement::SUPPORT_FOOT_DOUBLE;

      if (stepFeatures.leftfootHeight[i] < 1e-3)
	{
	  if (stepFeatures.rightfootHeight[i] < 1e-3)
	    // both feet on the floor
	    phase = WalkMovement::SUPPORT_FOOT_DOUBLE;
	  else
	    // left foot on the floor only
	    phase = WalkMovement::SUPPORT_FOOT_LEFT;
	}
      else
	{
	  if (stepFeatures.rightfootHeight[i] < 1e-3)
	    // right foot on the floor only
	    phase = WalkMovement::SUPPORT_FOOT_RIGHT;
	  else
	    // no foot on the floor !?
	    assert (0);
	}

      if (phase != oldPhase)
	{
	  oldPhase = phase;
	  trajectories_->supportFoot.push_back
	    (std::make_pair (i * STEP, phase));
	}
    }

  this->comOut_.recompute (0);
  this->zmpOut_.recompute (0);
  this->leftAnkleOut_.recompute (0);
  this->rightAnkleOut_.recompute (0);
}

void
FeetFollowerAnalyticalPg::pushStep (const ml::Vector& step)
{
  if (step.size () != 7)
    {
      std::cerr << "invalid step" << std::endl;
      return;
    }

  steps_.push_back (step);
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (FeetFollowerAnalyticalPg,
				    "FeetFollowerAnalyticalPg");


namespace command
{
  GenerateTrajectory::GenerateTrajectory (FeetFollowerAnalyticalPg& entity,
					  const std::string& docstring)
    : Command (entity, std::vector<Value::Type> (), docstring)
  {}

  Value GenerateTrajectory::doExecute()
  {
    FeetFollowerAnalyticalPg& entity =
      static_cast<FeetFollowerAnalyticalPg&>(owner ());
    entity.generateTrajectory ();
    return Value ();
  }

  PushStep::PushStep (FeetFollowerAnalyticalPg& entity,
		      const std::string& docstring)
    : Command (entity, boost::assign::list_of(Value::VECTOR), docstring)
  {}

  Value PushStep::doExecute()
  {
    FeetFollowerAnalyticalPg& entity =
      static_cast<FeetFollowerAnalyticalPg&>(owner ());

    std::vector<Value> values = getParameterValues ();
    ml::Vector step = values[0].value ();

    entity.pushStep (step);
    return Value ();
  }

  ClearSteps::ClearSteps (FeetFollowerAnalyticalPg& entity,
			const std::string& docstring)
    : Command (entity, std::vector<Value::Type> (), docstring)
  {}

  Value ClearSteps::doExecute()
  {
    FeetFollowerAnalyticalPg& entity =
      static_cast<FeetFollowerAnalyticalPg&>(owner ());
    entity.clearSteps ();
    return Value ();
  }
} // end of namespace command.
