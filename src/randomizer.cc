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

#include <boost/assign/list_of.hpp>
#include <boost/make_shared.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

#include "common.hh"
#include "randomizer.hh"

Randomizer::Randomizer (const std::string& name)
  : dg::Entity (name),
    signals_ (),
    gen_ ()
{
  std::string docstring;
  addCommand ("addSignal",
	      new command::AddSignal (*this, docstring));
}

Randomizer::~Randomizer ()
{}

void
Randomizer::addSignal (const std::string& name, unsigned size)
{
  if (signals_.find (name) != signals_.end ())
    throw std::runtime_error ("signal already exists");

  signalVector_t* ptr = 0;
  try
    {
      ptr = new signalVector_t
	(boost::bind (&Randomizer::computeSignal, this, _1, _2, name),
	 dg::sotNOSIGNAL,
	 MAKE_SIGNAL_STRING (name, false, "Vector", name));
    }
  catch (...)
    {
      delete ptr;
      throw;
    }

  signals_[name] =
    std::make_pair (boost::shared_ptr<signalVector_t> (ptr), size);

  signals_[name].first->setNeedUpdateFromAllChildren (true);
  signalRegistration (*signals_[name].first);
}

// FIXME:
// For now, the distribution, generators and seeds are hard-coded
// but it each signal should have its specific setup.
ml::Vector&
Randomizer::computeSignal (ml::Vector& res, int, const std::string& name)
{
  static const double mean = 0.;
  static const double standard_deviation = 1.;
  boost::normal_distribution<> dist (mean, standard_deviation);

  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> >
    die (gen_, dist);

  res.resize (signals_[name].second);
  for (unsigned i = 0; i < res.size (); ++i)
    res (i) =  die ();
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (Randomizer, "Randomizer");

namespace command
{
  AddSignal::AddSignal (Randomizer& entity, const std::string& docstring)
    : Command (entity,
	       boost::assign::list_of(Value::STRING) (Value::UNSIGNED),
	       docstring)
  {}

  Value AddSignal::doExecute ()
  {
    Randomizer& entity = static_cast<Randomizer&> (owner ());
    const std::vector<Value>& values = getParameterValues ();
    std::string name = values[0].value ();
    unsigned size = values[1].value ();
    entity.addSignal (name, size);
    return Value ();
  }
} // end of namespace command.
