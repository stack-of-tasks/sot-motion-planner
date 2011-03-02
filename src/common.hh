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
#ifndef SOT_MOTION_PLANNER_COMMON_HH
# define SOT_MOTION_PLANNER_COMMON_HH
# include <boost/format.hpp>

inline std::string
makeSignalString (const std::string& className,
		  const std::string& instanceName,
		  bool isInputSignal,
		  const std::string& signalType,
		  const std::string& signalName)
{
  boost::format fmt ("%s(%s)::%s(%s)::%s");
  fmt % className
    % instanceName
    % (isInputSignal ? "input" : "output")
    % signalType
    % signalName;
  return fmt.str ();
}

# define MAKE_SIGNAL_STRING(NAME, IS_INPUT, OUTPUT_TYPE, SIGNAL_NAME)	\
  makeSignalString (typeid (*this).name (), NAME,			\
		    IS_INPUT, OUTPUT_TYPE, SIGNAL_NAME)



# define INIT_SIGNAL_IN(SIGNAL_NAME, METHOD_NAME, TYPE)			\
  boost::bind(&Localizer::METHOD_NAME, this, _1, _2),			\
    dg::sotNOSIGNAL,							\
    MAKE_SIGNAL_STRING(name, true, TYPE, SIGNAL_NAME)

# define INIT_SIGNAL_OUT(SIGNAL_NAME, METHOD_NAME, TYPE)		\
  boost::bind(&Localizer::METHOD_NAME, this, _1, _2),			\
    dg::sotNOSIGNAL,							\
    MAKE_SIGNAL_STRING(name, false, TYPE, SIGNAL_NAME)

#endif //! SOT_MOTION_PLANNER_COMMON_HH
