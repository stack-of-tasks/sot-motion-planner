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

#include <cassert>
#include <string>

#include "error-trajectory.hh"

namespace sot
{
  ErrorTrajectory::ErrorTrajectory (const interval_t& range,
				    const vector_t& finalValue,
				    std::string name) throw ()
    : Trajectory (finalValue.size (), name),
      range_ (range),
      finalValue_ (finalValue),
      a_ (finalValue.size ()),
      b_ (finalValue.size ())
  {
    assert (getUpperBound (range) >= getLowerBound (range));
    const double& tmin = getLowerBound (range);
    const double& tmax = getUpperBound (range);
    const double T = tmax - tmin;
    const double T_2 = T * T;
    const double T_3 = T_2 * T;

    a_ = -2. / T_3 * finalValue;
    b_ = 3. * finalValue / T_2;
  }

  ErrorTrajectory::~ErrorTrajectory () throw ()
  {}

  void
  ErrorTrajectory::impl_compute (result_t& result, const value_type& t)
    const throw ()
  {
    assert (t >= getLowerBound (range_));
    assert (t <= getUpperBound (range_));

    const double t_ = t - getLowerBound (range_);
    const double t_2 = t_ * t_;
    const double t_3 = t_2 * t_;
      result = a_ * t_3 + b_ * t_2;
  }

} // end of namespace sot.
