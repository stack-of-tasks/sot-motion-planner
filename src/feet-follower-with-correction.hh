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

# include <sot/core/matrix-homogeneous.hh>

# include "feet-follower.hh"

class FeetFollowerWithCorrection : public FeetFollower
{
public:
  /// \brief Vector input signal.
  typedef dg::SignalPtr<ml::Vector, int> signalInVector_t;

  static const std::string CLASS_NAME;

  explicit FeetFollowerWithCorrection (const std::string& name);
  virtual ~FeetFollowerWithCorrection ();

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

protected:
  virtual void impl_update ();
  void updateCorrection ();

private:
  FeetFollower* referenceTrajectory_;

  signalInVector_t offsetIn_;
  sot::MatrixHomogeneous correction_;
};

#endif //! SOT_MOTION_PLANNER_CORRECTION_HH
