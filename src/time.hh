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
#ifndef SOT_MOTION_PLANNER_TIME_HH
# define SOT_MOTION_PLANNER_TIME_HH
# include <boost/date_time/posix_time/posix_time.hpp>

# include <jrl/mal/boost.hh>

namespace ml = ::maal::boost;

namespace sot
{
  namespace motionPlanner
  {

    void timestamp (ml::Vector& res);
    boost::posix_time::ptime timestampToDateTime (const ml::Vector& timestamp);

  } // end of namespace motionPlanner.
} // end of namespace sot.

#endif //! SOT_MOTION_PLANNER_TIME_HH
