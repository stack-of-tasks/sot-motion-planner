// Copyright 2011, Thomas Moulard, CNRS.
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

#ifndef SOT_MOTION_PLANNER_ERROR_TRAJECTORY_HH
# define SOT_MOTION_PLANNER_ERROR_TRAJECTORY_HH
# include <vector>

# include "trajectory.hh"

namespace sot
{
  class ErrorTrajectory : public Trajectory
  {
  public:
    typedef std::vector<vector_t> discretizedData_t;

    ErrorTrajectory (const interval_t& range,
		     const vector_t& finalValue,
		     std::string name) throw ();

    virtual ~ErrorTrajectory () throw ();

    virtual const interval_t& getRange () const
    {
      return range_;
    }
  private:
    virtual void impl_compute (result_t& result, const value_type& t)
      const throw ();

    interval_t range_;
    const vector_t finalValue_;
    vector_t a_;
    vector_t b_;
  };

} // end of namespace sot.

#endif //! SOT_MOTION_PLANNER_ERROR_TRAJECTORY_HH
