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

#ifndef SOT_MOTION_PLANNER_SUPERVISOR_HH
# define SOT_MOTION_PLANNER_SUPERVISOR_HH
# include <boost/tuple/tuple.hpp>

# include <dynamic-graph/command.h>
# include <dynamic-graph/entity.h>
# include <dynamic-graph/factory.h>
# include <dynamic-graph/null-ptr.hh>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>

# include <sot/core/feature-posture.h>
# include <sot/core/matrix-homogeneous.hh>
# include <sot/core/sot.hh>
# include <sot/core/task-abstract.hh>

# include "feet-follower.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

class Supervisor;

namespace command
{
  namespace supervisor
  {
    using ::dynamicgraph::command::Command;
    using ::dynamicgraph::command::Value;

    class SetSolver : public Command
    {
    public:
      SetSolver (Supervisor& entity,
		 const std::string& docstring);
      virtual Value doExecute ();
    };

    class SetPostureFeature : public Command
    {
    public:
      SetPostureFeature (Supervisor& entity,
			 const std::string& docstring);
      virtual Value doExecute ();
    };

    class AddTask : public Command
    {
    public:
      AddTask (Supervisor& entity,
	       const std::string& docstring);
      virtual Value doExecute ();
    };

    class AddFeetFollowerStartCall : public Command
    {
    public:
      AddFeetFollowerStartCall (Supervisor& entity,
				const std::string& docstring);
      virtual Value doExecute ();
    };

    class Display : public Command
    {
    public:
      Display (Supervisor& entity,
	       const std::string& docstring);
      virtual Value doExecute ();
    };

    class Start : public Command
    {
    public:
      Start (Supervisor& entity,
	    const std::string& docstring);
      virtual Value doExecute ();
    };

    class Stop : public Command
    {
    public:
      Stop (Supervisor& entity,
	    const std::string& docstring);
      virtual Value doExecute ();
    };
  } // end of namespace supervisor.
} // end of namespace command.

class Supervisor : public dg::Entity
{
  DYNAMIC_GRAPH_ENTITY_DECL ();
public:
  typedef boost::tuple<double, double, int, ml::Vector> taskData_t;
  typedef std::map<dynamicgraph::sot::TaskAbstract*, taskData_t> motions_t;

  typedef std::map<FeetFollower*, std::pair<double, bool> > startCalls_t;


  /// \name Constructor and destructor.
  /// \{
  explicit Supervisor (const std::string& name);
  virtual ~Supervisor ();
  /// \}

  void addTask (dynamicgraph::sot::TaskAbstract* task,
		double min, double max, int level,
		const ml::Vector& unlockedDofs);

  motions_t& motions ()
  {
    return motions_;
  }

  startCalls_t& startCalls ()
  {
    return startCalls_;
  }

  dynamicgraph::sot::Sot*& sot ()
  {
    return sot_;
  }

  dynamicgraph::sot::FeaturePosture*& featurePosture ()
  {
    return featurePosture_;
  }

  void setOrigin (const double& origin)
  {
    tOrigin_ = origin;
  }

  void addFeetFollowerStartCall (FeetFollower* ff, double t);

  void start ();
  void stop ();

protected:
  void updateFeetFollowerStartCall (const double&);
  void updateMotions (const double&);
  int& update (int&, int);

private:
  dg::SignalTimeDependent<int, int> trigger_;
  dynamicgraph::sot::Sot* sot_;
  dynamicgraph::sot::FeaturePosture* featurePosture_;
  double t_;
  double tOrigin_;
  motions_t motions_;
  startCalls_t startCalls_;
};

#endif //! SOT_MOTION_PLANNER_SUPERVISOR_HH
