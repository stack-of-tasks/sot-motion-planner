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

#ifndef SOT_MOTION_PLANNER_SET_REFERENCE_TRAJECTORY_HH
# define SOT_MOTION_PLANNER_SET_REFERENCE_TRAJECTORY_HH
# include <string>

# include <jrl/mal/boost.hh>

# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/pool.h>

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

class FeetFollower;

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  template <typename T>
  class SetReferenceTrajectory : public Command
  {
  public:
    SetReferenceTrajectory (T& entity,
			    const std::string& docstring);
    virtual Value doExecute ();
  };

  template <typename T>
  SetReferenceTrajectory<T>::SetReferenceTrajectory
  (T& entity, const std::string& docstring)
    : Command (entity, boost::assign::list_of (Value::STRING), docstring)
  {}

  template <typename T>
  Value SetReferenceTrajectory<T>::doExecute ()
  {
    T& entity = static_cast<T&> (owner ());

    std::vector<Value> values = getParameterValues ();
    std::string name = values[0].value ();

    FeetFollower* referenceTrajectory = 0;
    if (dynamicgraph::g_pool.existEntity (name))
      {
	referenceTrajectory =
	  dynamic_cast<FeetFollower*> (&dynamicgraph::g_pool.getEntity (name));
	if (!referenceTrajectory)
	  std::cerr << "entity is not a FeetFollower" << std::endl;
      }
    else
      std::cerr << "invalid entity name" << std::endl;

    entity.setReferenceTrajectory (referenceTrajectory);
    return Value ();
  }
} // end of namespace command.


#endif //! SOT_MOTION_PLANNER_SET_REFERENCE_TRAJECTORY_HH
