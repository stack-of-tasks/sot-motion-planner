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
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>

#include "common.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

class WaistYaw : public dg::Entity
{
 public:
  typedef dg::SignalPtr<ml::Vector, int> signalIn_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalOut_t;

  static const std::string CLASS_NAME;

  explicit WaistYaw (const std::string& name)
    : Entity(name),
      state_
      (dg::nullptr,
       MAKE_SIGNAL_STRING (name, true, "Vector", "state")),
      waistYaw_
      (dg::nullptr,
       MAKE_SIGNAL_STRING (name, true, "Vector", "waistYaw")),
      error_ (INIT_SIGNAL_OUT ("error", WaistYaw::updateError, "Vector")),
      sdes_ (INIT_SIGNAL_OUT ("sdes", WaistYaw::updateSdes, "Vector"))
  {
    signalRegistration (error_ << sdes_ << state_ << waistYaw_);

    error_.addDependency (state_);
    sdes_.addDependency (state_);
    error_.addDependency (waistYaw_);
    sdes_.addDependency (waistYaw_);
  }

  virtual ~WaistYaw ()
  {}

  virtual const std::string& getClassName () const
  {
    return CLASS_NAME;
  }

private:
  ml::Vector& updateError (ml::Vector& res, int t)
  {
    res.resize (3);
    res (0) = state_ (t) (3);
    res (1) = state_ (t) (4);
    res (2) = state_ (t) (5);
    return res;
  }

  ml::Vector& updateSdes (ml::Vector& res, int t)
  {
    res.resize (3);
    res (0) = 0.;
    res (1) = 0.;
    res (2) = waistYaw_ (t) (0);
    return res;
  }

  signalIn_t state_;
  signalIn_t waistYaw_;
  signalOut_t error_;
  signalOut_t sdes_;
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(WaistYaw, "WaistYaw");
