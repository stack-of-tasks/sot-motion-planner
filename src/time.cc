// Copyright 2011, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
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

#include <boost/numeric/conversion/converter.hpp>
#include <boost/date_time/date.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "time.hh"

namespace sot
{
  namespace motionPlanner
  {

    void timestamp (ml::Vector& res)
    {
      using namespace boost::gregorian;
      using namespace boost::posix_time;

      typedef boost::posix_time::ptime ptime_t;

      if (res.size () != 2)
	res.resize (2);

      ptime_t time =
	boost::posix_time::microsec_clock::universal_time ();

      ptime_t timet_start(date(1970,1,1));

      time_duration diff = time - timet_start;

      int64_t sec = diff.ticks() / time_duration::rep_type::res_adjust ();
      int64_t usec =
	diff.fractional_seconds();

      typedef boost::numeric::converter<double, int64_t> Int64_t2Double;

      res (0) = Int64_t2Double::convert (sec);
      res (1) = Int64_t2Double::convert (usec);
    }

    boost::posix_time::ptime
    timestampToDateTime (const ml::Vector& timestamp)
    {
      using namespace boost::gregorian;
      using namespace boost::posix_time;

      typedef boost::numeric::converter<int64_t, double> Double2Long;

      int64_t sec = Double2Long::convert (timestamp (0));
      int64_t usec = Double2Long::convert (timestamp (1));

      typedef boost::posix_time::ptime ptime_t;
      ptime_t time (date(1970,1,1),
		    seconds (sec) + microseconds (usec));
      return time;
    }
  } // end of namespace motionPlanner.
} // end of namespace sot.
