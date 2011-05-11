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
# include <cmath>
# include <boost/bind.hpp>
# include <boost/format.hpp>

# include <jrl/mathtools/angle.hh>
# include <sot/core/matrix-homogeneous.hh>
# include <sot/core/matrix-rotation.hh>
# include <sot/core/vector-roll-pitch-yaw.hh>

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
  boost::bind(&METHOD_NAME, this, _1, _2),				\
    dg::sotNOSIGNAL,							\
    MAKE_SIGNAL_STRING(name, true, TYPE, SIGNAL_NAME)

# define INIT_SIGNAL_OUT(SIGNAL_NAME, METHOD_NAME, TYPE)		\
  boost::bind(&METHOD_NAME, this, _1, _2),				\
    dg::sotNOSIGNAL,							\
    MAKE_SIGNAL_STRING(name, false, TYPE, SIGNAL_NAME)


namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;

namespace sot
{
  using namespace ::dynamicgraph::sot;
}

inline sot::MatrixHomogeneous
XYThetaToMatrixHomogeneous (const ml::Vector& xytheta)
{
  assert (xytheta.size () == 3);
  ml::Vector t (3);
  t (0) = xytheta (0);
  t (1) = xytheta (1);
  t (2) = 0.;

  jrlMathTools::Angle theta (xytheta (2));

  sot::VectorRollPitchYaw vR;
  vR (2) = theta.value ();
  sot::MatrixRotation R;
  vR.toMatrix (R);
  sot::MatrixHomogeneous res;
  res.buildFrom (R, t);
  return res;
}

inline sot::MatrixHomogeneous
XYZThetaToMatrixHomogeneous (const ml::Vector& xyztheta)
{
  assert (xyztheta.size () == 4);
  ml::Vector t (3);
  t (0) = xyztheta (0);
  t (1) = xyztheta (1);
  t (2) = xyztheta (2);

  jrlMathTools::Angle theta (xyztheta (3));

  sot::VectorRollPitchYaw vR;
  vR (2) = theta.value ();
  sot::MatrixRotation R;
  vR.toMatrix (R);
  sot::MatrixHomogeneous res;
  res.buildFrom (R, t);
  return res;
}


template <typename T>
sot::MatrixHomogeneous
XYThetaToMatrixHomogeneous (const T& xytheta)
{
  assert (xytheta.size () == 3);
  ml::Vector t (3);
  t (0) = xytheta[0];
  t (1) = xytheta[1];
  t (2) = 0.;

  jrlMathTools::Angle theta (xytheta[2]);

  sot::VectorRollPitchYaw vR;
  vR (2) = theta.value ();
  sot::MatrixRotation R;
  vR.toMatrix (R);
  sot::MatrixHomogeneous res;
  res.buildFrom (R, t);
  return res;
}

template <typename T>
ml::Vector
MatrixHomogeneousToXYTheta (const T& M)
{
  ml::Vector res (3);
  res (0) = M (0, 3);
  res (1) = M (1, 3);
  res (2) = atan2 (M (1, 0), M (0, 0));
  return res;
}


#endif //! SOT_MOTION_PLANNER_COMMON_HH
