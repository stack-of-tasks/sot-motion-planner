// Copyright 2011, Thomas Moulard, CNRS.
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

#include <cassert>
#include <cmath>
#include <fstream>
#include <vector>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/numeric/conversion/converter.hpp>
#include <boost/tokenizer.hpp>
#include <roboptim/core/function.hh>

#include "discretized-trajectory.hh"

namespace fs = boost::filesystem;

typedef boost::numeric::converter<unsigned, double> Double2Unsigned;

namespace sot
{
  DiscretizedTrajectory::DiscretizedTrajectory
  (const discreteInterval_t& range,
   const std::vector<vector_t>& data,
   std::string name) throw ()
    : Trajectory (data.empty () ? 0 : data[0].size (), name),
      range_ (range),
      discretizedData_ (data)
  {
    assert (getStep (range_) > 0.);
    assert (getUpperBound (range) >= getLowerBound (range));

#ifndef NDEBUG
    double l = getUpperBound (range) - getLowerBound (range);
    unsigned n = Double2Unsigned::convert (l / getStep (range_));

    assert (n * getStep (range_) - l < 1e-6);
    assert ((l / getStep (range_)) - data.size () < 1e-6);
#endif //! NDEBUG
  }

  DiscretizedTrajectory::DiscretizedTrajectory
  (const DiscretizedTrajectory& traj) throw ()
    : Trajectory (traj.outputSize (), traj.getName ()),
      range_ (traj.range_),
      discretizedData_ (traj.discretizedData_)
  {}

  boost::shared_ptr<DiscretizedTrajectory>
  DiscretizedTrajectory::loadTrajectoryFromFile (const fs::path& path,
						 const value_type& step,
						 const std::string& name)
  {
    using boost::tokenizer;
    using boost::lexical_cast;

    assert (fs::exists (path) && !fs::is_directory (path));

    std::ifstream file (path.string ().c_str ());

    std::vector<vector_t> data;
    size_t size = 0;

    while (file.good ())
      {
	std::string buffer;
	std::getline (file, buffer);

	if (buffer.empty ())
	  continue;

	boost::char_separator<char> sep(" \t");
	tokenizer<boost::char_separator<char> > tok (buffer, sep);
	if (!size)
	  BOOST_FOREACH (std::string s, tok)
	    ++size, s = s;
	else
	  {
	    size_t size_ = 0;
	    BOOST_FOREACH (std::string s, tok)
	      ++size_, s = s;
	    assert (size_ == size);
	  }

	vector_t result (size);
	unsigned i = 0;
	BOOST_FOREACH (const std::string& value, tok)
	  result[i++] = lexical_cast<double> (value);
	data.push_back (result);
      }

    discreteInterval_t range (0., data.size () * step, step);

    return boost::make_shared<DiscretizedTrajectory> (range, data, name);
  }


  DiscretizedTrajectory::~DiscretizedTrajectory () throw ()
  {}

  void
  DiscretizedTrajectory::impl_compute (result_t& result,
				       const value_type& t)
    const throw ()
  {
    assert (t >= getLowerBound (range_));
    assert (t <= getUpperBound (range_));

    unsigned idx = Double2Unsigned::convert (round (t / getStep(range_)));

    result = discretizedData_[idx];
  }
} // end of namespace sot.
