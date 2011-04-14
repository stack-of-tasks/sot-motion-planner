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

#ifndef SOT_MOTION_PLANNER_RANDOMIZER_HH
# define SOT_MOTION_PLANNER_RANDOMIZER_HH
# include <string>
# include <utility>

# include <boost/shared_ptr.hpp>
# include <boost/random/mersenne_twister.hpp>

# include <jrl/mal/boost.hh>

# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/pool.h>
# include <dynamic-graph/signal-time-dependent.h>

# include "common.hh"
# include "discretized-trajectory.hh"
# include "feet-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}


class Randomizer : public dg::Entity
{
 public:
  static const std::string CLASS_NAME;

  typedef dg::SignalTimeDependent<ml::Vector, int> signalVector_t;

  explicit Randomizer (const std::string& name);
  virtual ~Randomizer ();

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

  void addSignal (const std::string& name, unsigned size);

  ml::Vector& computeSignal (ml::Vector& res, int, const std::string& name);

protected:
  std::map<std::string, std::pair<boost::shared_ptr<signalVector_t>, unsigned> >
  signals_;
  boost::mt19937 gen_;
};

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  class AddSignal : public Command
  {
  public:
    AddSignal (Randomizer& entity, const std::string& docstring);
    virtual Value doExecute ();
  };
} // end of namespace command.


#endif //! SOT_MOTION_PLANNER_RANDOMIZER_HH
