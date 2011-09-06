// Copyright 2011, Thomas Moulard JRL, CNRS/AIST.
//
// This file is part of sot-motion-planner.
// sot-motion-planner is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// sot-motion-planner is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// sot-motion-planner. If not, see <http://www.gnu.org/licenses/>.

#ifndef SOT_MOTION_PLANNER_CORRECTION_HH
# define SOT_MOTION_PLANNER_CORRECTION_HH
# include <string>

# include <dynamic-graph/command.h>
# include <dynamic-graph/signal-ptr.h>
# include <sot/core/matrix-homogeneous.hh>

# include "error-trajectory.hh"
# include "feet-follower.hh"

# include "set-reference-trajectory.hh"

struct Correction
{
  explicit Correction
  (const sot::MatrixHomogeneous& positionError,
   const sot::ErrorTrajectory::interval_t& leftAnkleCorrectionInterval,
   const sot::ErrorTrajectory::interval_t& rightAnkleCorrectionInterval,
   const sot::ErrorTrajectory::interval_t& comCorrectionInterval,
   const sot::ErrorTrajectory::vector_t& error)
    : positionError (positionError),

      leftAnkleCorrection
      (leftAnkleCorrectionInterval,
       error, "left-ankle correction"),

      rightAnkleCorrection
      (rightAnkleCorrectionInterval,
       error, "right-ankle correction"),

      comCorrection
      (comCorrectionInterval,
       error, "com correction")
  {}

  sot::MatrixHomogeneous positionError;

  sot::ErrorTrajectory leftAnkleCorrection;
  sot::ErrorTrajectory rightAnkleCorrection;
  sot::ErrorTrajectory comCorrection;
};


/// \brief Decorator providing walking trajectory correction w.r.t an
///        estimation of the robot position.
///
/// FIXME: document.
class FeetFollowerWithCorrection : public FeetFollower
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  /// \brief Vector input signal.
  typedef dg::SignalPtr<ml::Vector, int> signalVectorIn_t;
  /// \brief Vector output signal.
  typedef dg::SignalTimeDependent<ml::Vector, int> signalVectorOut_t;

  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalMatrixHomoIn_t;

  explicit FeetFollowerWithCorrection (const std::string& name);
  virtual ~FeetFollowerWithCorrection ();

  virtual boost::optional<const WalkMovement&> walkMovement () const
  {
    if (!referenceTrajectory_)
      return boost::optional<const WalkMovement&> ();
    return referenceTrajectory_->walkMovement ();
  }

  void setReferenceTrajectory (FeetFollower* ptr);

  void setSafetyLimits (const double& maxErrorX,
			const double& maxErrorY,
			const double& maxErrorTheta)
  {
    maxErrorX_ = maxErrorX;
    maxErrorY_ = maxErrorY;
    maxErrorTheta_ = maxErrorTheta;
  }

  virtual void impl_start ();

  virtual boost::optional<int>
  trajectorySize () const
  {
    if (!referenceTrajectory_)
      return boost::optional<int> ();
    return referenceTrajectory_->trajectorySize ();
  }

  ml::Vector& footsteps ()
  {
    return footsteps_;
  }
  const ml::Vector& footsteps () const
  {
    return footsteps_;
  }

  double& footstepsTime ()
  {
    return footstepsTime_;
  }
  const double& footstepsTime () const
  {
    return footstepsTime_;
  }

protected:
  virtual void impl_update ();
  void updateCorrection ();
  void updateVelocities ();

  ml::Vector& updateDbgFootsteps (ml::Vector& res, int);
  void updateFootsteps (const double& time,
			const sot::MatrixHomogeneous& error);

  void computeNewCorrection ();

private:
  FeetFollower* referenceTrajectory_;

  signalVectorIn_t offsetIn_;
  signalMatrixHomoIn_t positionIn_;

  sot::MatrixHomogeneous correctionLeftAnkle_;
  sot::MatrixHomogeneous correctionRightAnkle_;
  sot::MatrixHomogeneous correctionCom_;

  std::vector<boost::shared_ptr<Correction> > corrections_;

  // Security limits.
  double maxErrorX_;
  double maxErrorY_;
  double maxErrorTheta_;

  double footstepsTime_;
  ml::Vector footsteps_;
  signalVectorOut_t dbgFootstepsOut_;
};

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  class SetSafetyLimits : public Command
  {
  public:
    SetSafetyLimits (FeetFollowerWithCorrection& entity,
		     const std::string& docstring);
    virtual Value doExecute ();
  };

  class SetFootsteps : public Command
  {
  public:
    SetFootsteps (FeetFollowerWithCorrection& entity,
		  const std::string& docstring);
    virtual Value doExecute ();
  };
} // end of namespace command.


#endif //! SOT_MOTION_PLANNER_CORRECTION_HH
