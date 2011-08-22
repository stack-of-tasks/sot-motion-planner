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

#include "time.hh"

int main()
{
  ml::Vector t(2);
  sot::motionPlanner::timestamp (t);

  boost::posix_time::ptime t2 = 
    sot::motionPlanner::timestampToDateTime(t);

  // Values should not be the same but should be near.
  std::cout << boost::posix_time::microsec_clock::universal_time () << std::endl;
  std::cout << t2 << std::endl;
}
