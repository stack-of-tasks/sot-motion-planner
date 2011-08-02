// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot/core/matrix-homogeneous.hh>

#include "common.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}


class PostureError : public dg::Entity
{
 public:
  typedef dg::SignalPtr<ml::Vector, int> signalIn_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalOut_t;

  static const std::string CLASS_NAME;

  explicit PostureError (const std::string& name)
    : Entity(name),
      state_
      (dg::nullptr,
       MAKE_SIGNAL_STRING (name, true, "Vector", "state")),
      error_ (INIT_SIGNAL_OUT ("error", PostureError::updateError, "Vector"))
  {
    signalRegistration (error_ << state_);

    error_.addDependency (state_);
  }

  virtual ~PostureError ()
  {}

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

private:
  ml::Vector& updateError (ml::Vector& res, int t)
  {
    ml::Vector state = state_ (t);

    int errorSize = state.size () - 12 - 3 -1;
    if (errorSize < 0)
      return res;

    res.resize (errorSize);

    res (0) = state (3);
    res (1) = state (4);
    //res (2) = state (5);

    for (unsigned i = 0; i < errorSize - 2u; ++i)
      res (i + 2) = state (i + 6 + 12);

    return res;
  }

  signalIn_t state_;
  signalOut_t error_;
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PostureError, "PostureError");
