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

#include <boost/assign.hpp>

#include "error-merger.hh"

namespace command
{
  namespace errorMerger
  {
    AddErrorEstimation::AddErrorEstimation (ErrorMerger& entity,
					    const std::string& docstring)
      : Command
	(entity,
	 boost::assign::list_of (Value::STRING),
	 docstring)
    {}

    Value
    AddErrorEstimation::doExecute ()
    {
      ErrorMerger& errorMerger = static_cast<ErrorMerger&> (owner ());
      const std::vector<Value>& values = getParameterValues();
      const std::string& errorName = values[0].value();

      ErrorMerger::signalVectorIn_t* signal =
	new ErrorMerger::signalVectorIn_t
	(dg::nullptr,
	 MAKE_SIGNAL_STRING
	 (errorMerger.getName(), true, "Vector", "error_" + errorName));

      ErrorMerger::signalVectorIn_t* signalWeight =
	new ErrorMerger::signalVectorIn_t
	(dg::nullptr,
	 MAKE_SIGNAL_STRING
	 (errorMerger.getName(), true, "Vector", "weight_" + errorName));


      errorMerger.errorsIn ().push_back
	(std::make_pair
	 (boost::shared_ptr<ErrorMerger::signalVectorIn_t> (signal),
	  boost::shared_ptr<ErrorMerger::signalVectorIn_t> (signalWeight)));
      errorMerger.signalRegistration (*signal << *signalWeight);

      errorMerger.errorOut ().addDependency (*signal);
      errorMerger.errorOut ().addDependency (*signalWeight);

      return Value ();
    }
  } // end of namespace errorMerger.
} // end of namespace command.


ErrorMerger::ErrorMerger (const std::string& name)
  : dg::Entity (name),
    errorsIn_ (),
    errorOut_ (INIT_SIGNAL_OUT ("error", ErrorMerger::updateError, "Vector"))
{
  signalRegistration (errorOut_);
  errorOut_.setNeedUpdateFromAllChildren (true);

  ml::Vector zero (3);
  zero.setZero ();
  errorOut_.setConstant(zero);

  std::string docstring;
  addCommand
    ("addErrorEstimation",
     new command::errorMerger::AddErrorEstimation (*this, docstring));

}

ErrorMerger::~ErrorMerger ()
{}

ml::Vector&
ErrorMerger::updateError (ml::Vector& res, int t)
{
  if (res.size () != 3)
    res.resize (3);
  res.setZero ();
  const unsigned N = errorsIn_.size ();

  double M = 0;

  for (unsigned i = 0; i < N; ++i)
    {
      double w = 0.;
      try
	{
	  w = (*errorsIn_[i].second) (t) (0);
	}
      catch (...)
	{
	  std::cerr << "warning: weight not plugged" << std::endl;
	}

      if (w >= 1e-6)
	{
	  M += w;
	  res += w * (*errorsIn_[i].first) (t);
	}
    }

  if (M < 1e-6)
    {
      res.setZero ();
      return res;
    }

  for (unsigned i = 0; i < 3; ++i)
    res (i) /= M;
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (ErrorMerger, "ErrorMerger");
