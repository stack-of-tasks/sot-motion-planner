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

#ifndef SOT_MOTION_PLANNER_ERROR_MERGER_HH
# define SOT_MOTION_PLANNER_ERROR_MERGER_HH
# include <boost/shared_ptr.hpp>

# include <jrl/mal/boost.hh>

# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/signal-ptr.h>

# include <sot/core/matrix-homogeneous.hh>

# include "common.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

class ErrorMerger;

namespace command
{
  namespace errorMerger
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class AddErrorEstimation : public Command
    {
    public:
      AddErrorEstimation (ErrorMerger& entity,
			  const std::string& docstring);
      virtual Value doExecute ();
    };
  } // end of namespace errorMerger.
} // end of namespace command.

class ErrorMerger : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
 public:
  friend class command::errorMerger::AddErrorEstimation;

  /// \brief Input vector signal.
  typedef dg::SignalPtr<ml::Vector, int> signalVectorIn_t;
  /// \brief Output vector signal.
  typedef dg::SignalTimeDependent<ml::Vector, int> signalVectorOut_t;

  /// \name Constructor and destructor.
  /// \{
  explicit ErrorMerger (const std::string& name);
  virtual ~ErrorMerger ();
  /// \}

  std::vector<std::pair<boost::shared_ptr<signalVectorIn_t>,
			boost::shared_ptr<signalVectorIn_t> > >&
  errorsIn ()
  {
    return errorsIn_;
  }

protected:
  /// \brief Update the error signal.
  ml::Vector& updateError (ml::Vector& res, int);

private:
  std::vector<std::pair<boost::shared_ptr<signalVectorIn_t>,
			boost::shared_ptr<signalVectorIn_t> > > errorsIn_;
  signalVectorOut_t errorOut_;
};

#endif //! SOT_MOTION_PLANNER_ERROR_MERGER_HH
