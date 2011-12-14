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

#define protected public
# include <sot/core/sot.hh>
#undef protected


#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/pool.h>

#include "common.hh"
#include "supervisor.hh"
#include "time.hh"

static const double STEP = 5e-3; //FIXME:

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

    SetPostureFeature::SetPostureFeature (Supervisor& entity,
		 const std::string& docstring)
      : Command (entity, boost::assign::list_of (Value::STRING), docstring)
    {}

    Value
    SetPostureFeature::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());

      std::vector<Value> values = getParameterValues ();
      std::string name = values[0].value ();

      dynamicgraph::sot::FeaturePosture* featurePosture = 0;
      if (dynamicgraph::g_pool.existEntity (name))
	{
	  featurePosture =
	    dynamic_cast<dynamicgraph::sot::FeaturePosture*>
	    (&dynamicgraph::g_pool.getEntity (name));
	  if (!featurePosture)
	    std::cerr << "entity is not a feature posture entity" << std::endl;
	}
      else
	std::cerr << "invalid entity name" << std::endl;
      entity.featurePosture () = featurePosture;
      return Value ();
    }



    AddTask::AddTask (Supervisor& entity,
		      const std::string& docstring)
      : Command (entity, boost::assign::list_of
		 (Value::STRING)
		 (Value::DOUBLE) (Value::DOUBLE)
		 (Value::INT)
		 (Value::VECTOR),
		 docstring)
    {}

    Value
    AddTask::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());

      std::vector<Value> values = getParameterValues ();
      std::string name = values[0].value ();
      double min = values[1].value ();
      double max = values[2].value ();
      int level = values[3].value ();
      ml::Vector unlockedDofs = values[4].value ();

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
	entity.addTask (task, min, max, level, unlockedDofs);
      return Value ();
    }

    AddFeetFollowerStartCall::AddFeetFollowerStartCall (Supervisor& entity,
							const std::string& docstring)
      : Command (entity, boost::assign::list_of
		 (Value::STRING)
		 (Value::DOUBLE),
		 docstring)
    {}

    Value
    AddFeetFollowerStartCall::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());

      std::vector<Value> values = getParameterValues ();
      std::string name = values[0].value ();
      double time = values[1].value ();

      FeetFollower* ff = 0;
      if (dynamicgraph::g_pool.existEntity (name))
	{
	  ff =
	    dynamic_cast<FeetFollower*>
	    (&dynamicgraph::g_pool.getEntity (name));
	  if (!ff)
	    std::cerr << "entity is not a FeetFollower entity" << std::endl;
	}
      else
	std::cerr << "invalid entity name" << std::endl;

      if (ff)
	entity.addFeetFollowerStartCall (ff, time);
      std::cout << "feet follower start call registered" << std::endl;
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
      dynamicgraph::sot::TaskAbstract*, Supervisor::taskData_t> pair_t;

      boost::format fmt ("- %s [%f; %f] level = %d\n");
      BOOST_FOREACH (const pair_t& e, entity.motions ())
	{
	  fmt
	    % (e.first ? e.first->getName () : "invalid task")
	    % boost::get<0> (e.second) % boost::get<1> (e.second)
	    % boost::get<2> (e.second);
	  str += fmt.str ();
	}

      if (str.empty ())
	str = "no tasks in supervisor";

      return Value (str);
    }

    Start::Start (Supervisor& entity,
		      const std::string& docstring)
      : Command (entity, std::vector<Value::Type> (), docstring)
    {}

    Value
    Start::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());
      entity.start ();
      return Value ();
    }

    Stop::Stop (Supervisor& entity,
		      const std::string& docstring)
      : Command (entity, std::vector<Value::Type> (), docstring)
    {}

    Value
    Stop::doExecute ()
    {
      Supervisor& entity = static_cast<Supervisor&> (owner ());
      entity.stop ();
      return Value ();
    }


  } // end of namespace supervisor.
} // end of namespace command.


Supervisor::Supervisor (const std::string& name)
  : dg::Entity (name),
    trigger_ (INIT_SIGNAL_IN
	      ("trigger",
	       Supervisor::update, "Int")),
    sot_ (dg::nullptr),
    t_ (-1.),
    tOrigin_ (-1.),
    motions_ (),
    startCalls_ ()
{
  signalRegistration (trigger_);
  trigger_.setNeedUpdateFromAllChildren (true);

  std::string docstring;

  addCommand ("setSolver",
	      new command::supervisor::SetSolver (*this, docstring));
  addCommand ("setPostureFeature",
	      new command::supervisor::SetPostureFeature (*this, docstring));
  addCommand ("addTask",
	      new command::supervisor::AddTask (*this, docstring));
  addCommand ("addFeetFollowerStartCall",
	      new command::supervisor::AddFeetFollowerStartCall
	      (*this, docstring));

  addCommand ("start",
	      new command::supervisor::Start (*this, docstring));
  addCommand ("stop",
	      new command::supervisor::Stop (*this, docstring));

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
  t_ = t * STEP;
  if (tOrigin_ < 0.)
    return dummy;

  double tCurrent = (t * STEP) - tOrigin_;

  updateFeetFollowerStartCall (tCurrent);
  updateMotions (tCurrent);
  return dummy;
}

void
Supervisor::updateFeetFollowerStartCall (const double& t_)
{
  if (tOrigin_ < 0.)
    return;

  typedef std::pair<FeetFollower* const, std::pair<double, bool> > pair_t;
  BOOST_FOREACH (pair_t& e, startCalls_)
    {
      if (t_ >= e.second.first && !e.second.second)
	{
	  std::cout
	    << "starting "
	    << (e.first ? e.first->getName () : "invalid name")
	    << std::endl;
	  e.second.second = true;
	  e.first->start ();
	}
    }
}

void
Supervisor::updateMotions (const double& t_)
{
  if (!sot_)
    return;
  if (!featurePosture_)
    return;
  if (tOrigin_ < 0.)
    return;

  typedef std::pair<
  dynamicgraph::sot::TaskAbstract*, taskData_t> pair_t;

  BOOST_FOREACH (const pair_t& e, motions_)
    {
      dynamicgraph::sot::TaskAbstract* task = e.first;
      const taskData_t& taskData = e.second;
      const double& min = boost::get<0> (taskData);
      const double& max = boost::get<1> (taskData);
      const int& level = boost::get<2> (taskData);
      const ml::Vector& unlockedDofs = boost::get<3> (taskData);

      if (!task)
	continue;

      if (t_ < min || t_ > max)
	{
	  if (sot_->exist (*task))
	    {
	      sot_->remove (*task);
	      std::cout << "Removing " << task->getName () << std::endl;

	      // Lock dofs.
	      for (unsigned i = 0; i < unlockedDofs.size (); ++i)
		featurePosture_->selectDof ((int)unlockedDofs (i), true);
	    }
	}
      else
	if (!sot_->exist (*task))
	  {
	    sot_->push (*task);

	    std::cout << "Adding " << task->getName ()
		      << ", level = " << level << std::endl;

	    // Free dofs.
	    for (unsigned i = 0; i < unlockedDofs.size (); ++i)
	      featurePosture_->selectDof ((int)unlockedDofs (i), false);

	    // We compute how many "up" are required to ensure task
	    // priority.
	    unsigned nbUp = 0;

	    const dynamicgraph::sot::Sot::StackType& stack = sot_->stack;
	    typedef dynamicgraph::sot::Sot::StackType::const_iterator iter_t;

	    typedef motions_t::const_iterator motionsIter_t;

	    for (iter_t it = stack.begin (); it != stack.end (); ++it)
	      {
		motionsIter_t motionIt = motions_.find (*it);

		// Unknown task (not managed by this supervisor)
		if (motionIt == motions_.end ())
		  {
		    ++nbUp;
		    continue;
		  }

		// Task being inserted.
		if (motionIt->first == task)
		  {
		    ++nbUp;
		    continue;
		  }

		// Another task.
		const int& taskLevel = boost::get<2> (motionIt->second);

		// We have a higher priority than the current task.
		if (taskLevel < level)
		  ++nbUp;
	      }

	    for (unsigned i = 0; i < nbUp; ++i)
	      sot_->up (*task);
	  }
    }
}

void
Supervisor::addTask (dynamicgraph::sot::TaskAbstract* task,
		     double min, double max, int level,
		     const ml::Vector& unlockedDofs)
{
  if (!task)
    throw std::runtime_error ("invalid task");
  if (!sot_)
    throw std::runtime_error ("solver must be set before adding a task");

  motions ()[task] = Supervisor::taskData_t (min, max, level, unlockedDofs);
}

void
Supervisor::addFeetFollowerStartCall (FeetFollower* ff, double t)
{
  startCalls ()[ff] = std::make_pair (t, false);
}

void
Supervisor::stop ()
{
  motions_.clear ();
  startCalls_.clear ();
  tOrigin_ = -1;
  sot_ = 0;
  featurePosture_ = 0;
  std::cout << "supervisor stopped" << std::endl;
}

void
Supervisor::start ()
{
  if (tOrigin_ < 0.)
    tOrigin_ = std::max (t_, 0.);
  std::cout << "supervisor started at t = " << tOrigin_ << std::endl;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (Supervisor, "Supervisor");
