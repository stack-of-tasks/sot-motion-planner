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

#ifndef SOT_MOTION_PLANNER_DISCRETIZED_TRAJECTORY_HH
# define SOT_MOTION_PLANNER_DISCRETIZED_TRAJECTORY_HH
# include <boost/filesystem.hpp>
# include <boost/shared_ptr.hpp>

# include "trajectory.hh"

namespace sot
{
  class DiscretizedTrajectory : public Trajectory
  {
  public:
    typedef std::vector<vector_t> discretizedData_t;

    DiscretizedTrajectory (const discreteInterval_t& range,
			   const std::vector<vector_t>& data,
			   std::string name) throw ();

    DiscretizedTrajectory (const DiscretizedTrajectory& traj) throw ();

    static boost::shared_ptr<DiscretizedTrajectory>
    loadTrajectoryFromFile (const boost::filesystem::path& path,
			    const value_type& step,
			    const std::string& name);

    virtual ~DiscretizedTrajectory () throw ();

    const discreteInterval_t& getRange () const
    {
      return range_;
    }

    size_t trajectorySize () const
    {
      return discretizedData_.size ();
    }

  private:
    virtual void impl_compute (result_t& result, const value_type& t)
      const throw ();

    /// \brief Forbid explicitely to use the assigment operator.
    ///
    /// This method is not implemented and it is normal.
    /// See boost/noncopyable.hpp for instance for more information.
    const DiscretizedTrajectory& operator= (const DiscretizedTrajectory&);

    discreteInterval_t range_;
    discretizedData_t discretizedData_;
  };

} // end of namespace sot.

#endif //! SOT_MOTION_PLANNER_DISCRETIZED_TRAJECTORY_HH
