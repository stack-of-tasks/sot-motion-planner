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

#ifndef SOT_MOTION_PLANNER_TRAJECTORY_HH
# define SOT_MOTION_PLANNER_TRAJECTORY_HH
# include <cassert>
# include <string>
# include <roboptim/core/function.hh>

namespace sot
{
  class Trajectory : public roboptim::Function
  {
  public:
    using roboptim::Function::operator ();

    void operator () (result_t& result, const value_type& t) const throw ()
    {
      assert (isValidResult (result));
      this->impl_compute (result, t);
      assert (isValidResult (result));
    }

    result_t operator () (const value_type& t) const throw ()
    {
      result_t result (outputSize ());
      result.clear ();
      (*this) (result, t);
      return result;
    }

  protected:
    Trajectory (size_type outputSize, std::string name) throw ()
      : roboptim::Function (1, outputSize, name)
    {}

    virtual ~Trajectory () throw ()
    {}

    virtual void impl_compute (result_t& result, const argument_t& argument)
      const throw ()
    {
      assert (argument.size () == 1);
      this->impl_compute (result, argument[0]);
    }

    virtual void impl_compute (result_t& result, const value_type& t)
      const throw () = 0;
  };
} // end of namespace sot.

#endif //! SOT_MOTION_PLANNER_TRAJECTORY_HH
