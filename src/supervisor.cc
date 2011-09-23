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

#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/pool.h>

#include "common.hh"
#include "supervisor.hh"
#include "time.hh"

namespace command
{
  namespace supervisor
  {
    SetSolver::SetSolver (Supervisor& entity,
		 const std::string& docstring)
      : Command (entity, boost::assign::list_of (Value::STRING), docstring)
    {}

    Value
    SetSolver::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());

      std::vector<Value> values = getParameterValues ();
      std::string name = values[0].value ();

      dynamicgraph::sot::Sot* sot = 0;
      if (dynamicgraph::g_pool.existEntity (name))
	{
	  sot =
	    dynamic_cast<dynamicgraph::sot::Sot*>
	    (&dynamicgraph::g_pool.getEntity (name));
	  if (!sot)
	    std::cerr << "entity is not a stack of tasks entity" << std::endl;
	}
      else
	std::cerr << "invalid entity name" << std::endl;
      entity.sot () = sot;
      return Value ();
    }


    AddTask::AddTask (Supervisor& entity,
		      const std::string& docstring)
      : Command (entity, boost::assign::list_of
		 (Value::STRING) (Value::DOUBLE) (Value::DOUBLE), docstring)
    {}

    Value
    AddTask::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());

      std::vector<Value> values = getParameterValues ();
      std::string name = values[0].value ();
      double min = values[1].value ();
      double max = values[2].value ();

      dynamicgraph::sot::TaskAbstract* task = 0;
      if (dynamicgraph::g_pool.existEntity (name))
	{
	  task =
	    dynamic_cast<dynamicgraph::sot::TaskAbstract*>
	    (&dynamicgraph::g_pool.getEntity (name));
	  if (!task)
	    std::cerr << "entity is not a task abstract entity" << std::endl;
	}
      else
	std::cerr << "invalid entity name" << std::endl;

      if (task)
	entity.motions ()[task] = std::make_pair (min, max);
      return Value ();
    }

    Display::Display (Supervisor& entity,
		      const std::string& docstring)
      : Command (entity, std::vector<Value::Type> (), docstring)
    {}

    Value
    Display::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());
      std::string str;

      typedef std::pair<
      dynamicgraph::sot::TaskAbstract*, Supervisor::bounds_t> pair_t;

      boost::format fmt ("- %s [%f; %f]\n");
      BOOST_FOREACH (const pair_t& e, entity.motions ())
	{
	  fmt
	    % (e.first ? e.first->getName () : "invalid task")
	    % e.second.first % e.second.second;
	  str += fmt.str ();
	}

      if (str.empty ())
	str = "no tasks in supervisor";

      return Value (str);
    }

  } // end of namespace supervisor.
} // end of namespace command.


Supervisor::Supervisor (const std::string& name)
  : dg::Entity (name),
    trigger_ (INIT_SIGNAL_IN
	      ("trigger",
	       Supervisor::update, "Int")),
    sot_ (dg::nullptr),
    tOrigin_ (-1.),
    motions_ ()
{
  signalRegistration (trigger_);
  trigger_.setNeedUpdateFromAllChildren (true);

  std::string docstring;

  addCommand ("setSolver",
	      new command::supervisor::SetSolver (*this, docstring));
  addCommand ("addTask",
	      new command::supervisor::AddTask (*this, docstring));

  addCommand ("display",
	      new command::supervisor::Display (*this, docstring));

  using ::dynamicgraph::command::Setter;
  addCommand ("setOrigin", new Setter<Supervisor, double>
	      (*this, &Supervisor::setOrigin, docstring));
}

Supervisor::~Supervisor ()
{}

int&
Supervisor::update (int& dummy, int t)
{
  if (!sot_)
    return dummy;
  if (tOrigin_ < 0.)
    return dummy;

  static const double STEP = 5e-3;
  double t_ = (t - tOrigin_) * STEP;

  typedef std::pair<
  dynamicgraph::sot::TaskAbstract*, bounds_t> pair_t;

  BOOST_FOREACH (const pair_t& e, motions_)
    {
      dynamicgraph::sot::TaskAbstract* task = e.first;
      const bounds_t& bounds = e.second;
      const double& min = bounds.first;
      const double& max = bounds.second;

      if (!task)
	continue;

      if (t_ < min || t_ > max)
	{
	  if (sot_->exist (*task))
	    {
	      sot_->remove (*task);
	      std::cout << "Removing " << task->getName () << std::endl;
	    }
	}
      else
	if (!sot_->exist (*task))
	  {
	    sot_->push (*task);
	    std::cout << "Adding " << task->getName () << std::endl;
	    //sot_->up (*task);
	  }
    }
  return dummy;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (Supervisor, "Supervisor");
