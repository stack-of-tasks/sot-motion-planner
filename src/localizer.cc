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

#include <string>
#include <fstream>

#include <boost/assign/list_of.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/make_shared.hpp>

#include <jrl/mal/boost.hh>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <sot-core/matrix-homogeneous.h>
#include <sot-core/matrix-rotation.h>
#include <sot-core/vector-roll-pitch-yaw.h>

#include <dynamic-graph/command.h>


// Python:
//
//   blobObserver = LandmarkObservation()
//   blobObserver.setFeatureTrajectory((0, 0, 0, 0, ...))
//
//
// l = Localizer()
//
// l.addLandmarkObservation(name)
//
//
//
//

namespace ml = maal::boost;
namespace dg = dynamicgraph;


std::string
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

#define MAKE_SIGNAL_STRING(NAME, IS_INPUT, OUTPUT_TYPE, SIGNAL_NAME)	\
  makeSignalString (typeid (*this).name (), NAME,			\
		    IS_INPUT, OUTPUT_TYPE, SIGNAL_NAME)



#define INIT_SIGNAL_IN(SIGNAL_NAME, METHOD_NAME, TYPE)			\
  boost::bind(&Localizer::METHOD_NAME, this, _1, _2),			\
    dg::sotNOSIGNAL,							\
    MAKE_SIGNAL_STRING(name, true, TYPE, SIGNAL_NAME)

#define INIT_SIGNAL_OUT(SIGNAL_NAME, METHOD_NAME, TYPE)			\
  boost::bind(&Localizer::METHOD_NAME, this, _1, _2),			\
    dg::sotNOSIGNAL,							\
    MAKE_SIGNAL_STRING(name, false, TYPE, SIGNAL_NAME)


class Localizer;

namespace command
{
  class AddLandmarkObservation;
} // end of namespace command.

struct LandmarkObservation
{
  typedef dg::SignalPtr<sot::MatrixHomogeneous, int> signalInMatrixHomo_t;
  typedef dg::SignalPtr<ml::Matrix, int> signalInMatrix_t;
  typedef dg::SignalPtr<ml::Vector, int> signalInVector_t;

  explicit LandmarkObservation (Localizer& localizer,
				const std::string& signalNamePrefix);

  /// \name Output signals
  /// \{

  /// \brief Homogeneous matrix
  /// sensorPosition_ = s_i(q)
  signalInMatrixHomo_t sensorPosition_;

  /// \brief Variation of the sensor position w.r.t. the robot
  ///        configuration.
  ///
  /// JsensorPosition_ = \frac{ds_i}{dq}
  ///
  ///       s_i(q)[0] (x) s_i(q)[0] (y) s_i(q)[0] (z) ...
  /// q0[0]
  /// q0[1]
  /// q0[2]
  ///
  /// Here only (x,y, theta) columns are interesting.signalInVector_t
  signalInMatrix_t JsensorPosition_;


  /// \brief Expected current position of the feature.
  ///
  /// Read from motion plan.
  signalInVector_t featureReferencePosition_;

  /// \brief Variation of the feature position w.r.t the sensor
  ///        position.
  ///
  /// JfeatureReferencePosition_ = \frac{dP_{S_i,L_j}}{ds}(s_i(q0), l_j)
  ///
  ///          feature_x feature_y feature_z
  /// sensor_x
  /// sensor_y
  /// sensor_z
  signalInMatrix_t JfeatureReferencePosition_;

  /// \brief Current real feature position as detected by the sensor.
  signalInVector_t featureObservedPosition_;

  /// \brief Current weight of this landmark observation.
  /// This is a vector of N elements, where N is the dimension
  /// of the feature space.
  signalInVector_t weight_;
  /// \}

};

namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  // Command SetFiles
  struct AddLandmarkObservation : public Command
  {
    virtual ~AddLandmarkObservation ()
    {}

    explicit AddLandmarkObservation
    (Localizer& entity, const std::string& docstring);

    virtual Value doExecute ();
  };
} // end of namespace command.


namespace ublas = boost::numeric::ublas;
class Localizer : public dg::Entity
{
public:

  typedef dg::SignalTimeDependent<ml::Vector, int> signalOutVector_t;

  explicit Localizer (const std::string& name);

  size_t getFinalProblemSize (int t) const
  {
    size_t acc = 0;
    BOOST_FOREACH (const boost::shared_ptr<LandmarkObservation> obs,
		   this->landmarkObservations_)
      acc += obs->featureReferencePosition_ (t).size ();
    return acc;
  }

  ublas::vector<double> computeFeatureOffset (int t)
  {
    using namespace boost::numeric::ublas;

    size_t size = getFinalProblemSize (t);
    vector<double> featureDelta (size);
    // Compute im - im0.
    unsigned i = 0;
    BOOST_FOREACH (const boost::shared_ptr<LandmarkObservation> obs,
		   this->landmarkObservations_)
      {
	const vector<double>& featureObservedPos =
	  obs->featureObservedPosition_ (t).accessToMotherLib ();
	const vector<double>& featureReferencePos =
	  obs->featureReferencePosition_ (t).accessToMotherLib ();
	const vector<double>& weight =
	  obs->weight_ (t).accessToMotherLib ();

	range r (i, obs->featureReferencePosition_ (t).size ());
	vector_range<vector<double> > vr (featureDelta, r);
	vr = featureObservedPos - featureReferencePos;

	// Multiply by weight.
	for (unsigned idx = 0; idx < weight.size (); ++idx)
	  vr[idx] *= weight[idx];


	i += obs->featureReferencePosition_ (t).size ();
      }
    return featureDelta;
  }

  ublas::matrix<double> computeW (int t)
  {
    using namespace boost::numeric::ublas;

    matrix<double> W (getFinalProblemSize (t), 3);

    unsigned i = 0;
    BOOST_FOREACH (const boost::shared_ptr<LandmarkObservation> obs,
		   this->landmarkObservations_)
      {
	const matrix<double>& JfeatureReferencePosition =
	  obs->JfeatureReferencePosition_ (t).accessToMotherLib ();

	const matrix<double>& JsensorPosition =
	  obs->JsensorPosition_ (t).accessToMotherLib ();

	const vector<double>& weight =
	  obs->weight_ (t).accessToMotherLib ();

	range rx (i, obs->featureReferencePosition_ (t).size ());
	range ry (0, 3);
	matrix_range<matrix<double> > mr (W, rx, ry);

	mr = prod (JfeatureReferencePosition, JsensorPosition);

	// Multiply by weight.
	for (unsigned idx = 0; idx < weight.size (); ++idx)
	  matrix_row<matrix_range<matrix<double> > > (mr, idx)
	    *= weight[idx];

	i += obs->featureReferencePosition_ (t).size ();
      }
    return W;
  }

  ml::Vector& computeConfigurationOffset (ml::Vector& res, int t)
  {
    using namespace boost::numeric::ublas;
    vector<double> featureDelta = computeFeatureOffset (t);
    matrix<double> W = computeW (t);
    matrix<double> Wp = W; //FIXME: pseudoinverse here.
    return res.initFromMotherLib (prod (W, featureDelta));
  }

private:
  friend class LandmarkObservation;
  friend class command::AddLandmarkObservation;

  /// Commands
  /// - add_landmark_observation(name)

  /// \name Output signals
  /// \{
  signalOutVector_t configurationOffset_;
  /// \}

  std::vector<boost::shared_ptr<LandmarkObservation> > landmarkObservations_;
};


LandmarkObservation::LandmarkObservation (Localizer& localizer,
					  const std::string& signalNamePrefix)
  : sensorPosition_
    (dg::nullptr,
     MAKE_SIGNAL_STRING
     (localizer.getName (), true, "MatrixHomo", signalNamePrefix + "_sensorPosition")),
    JsensorPosition_
    (dg::nullptr,
     MAKE_SIGNAL_STRING
     (localizer.getName (), true, "Matrix", signalNamePrefix + "_JsensorPosition")),
    featureReferencePosition_
    (dg::nullptr,
     MAKE_SIGNAL_STRING
     (localizer.getName (), true, "Vector", signalNamePrefix + "_featureReferencePosition")),
    JfeatureReferencePosition_
    (dg::nullptr,
     MAKE_SIGNAL_STRING
     (localizer.getName (), true, "Matrix", signalNamePrefix + "_JfeatureReferencePosition")),
    featureObservedPosition_
    (dg::nullptr,
     MAKE_SIGNAL_STRING
     (localizer.getName (), true, "Vector", signalNamePrefix + "_featureObservedPosition")),
    weight_
    (dg::nullptr,
     MAKE_SIGNAL_STRING
     (localizer.getName (), true, "Vector", signalNamePrefix + "_weight"))
{
  localizer.signalRegistration (sensorPosition_
				<< JsensorPosition_
				<< featureReferencePosition_
				<< JfeatureReferencePosition_
				<< featureObservedPosition_
				<< weight_);
}

Localizer::Localizer (const std::string& name)
  : Entity(name),
    configurationOffset_
    (boost::bind (&Localizer::computeConfigurationOffset, this, _1, _2),
     dg::sotNOSIGNAL,
     MAKE_SIGNAL_STRING (name, false, "Vector", "")),
    landmarkObservations_ ()
{
  signalRegistration (configurationOffset_);

  std::string docstring = "    \n"
    "    Add a landmark observation to the localizer.\n"
    "    \n"
    "    In practice, calling this generates signals prefixed by\n"
    "    the name of the landmark observation.\n"
    "    \n"
    "    Generated signals are:"
    "    - FIXME"
    "    \n"
    "      Input:\n"
    "        - a string: landmark observation name,\n"
    "      Return:\n"
    "        - nothing\n";
  addCommand
    ("add_landmark_observation",
     new command::AddLandmarkObservation (*this, docstring));
}

namespace command
{
  AddLandmarkObservation::AddLandmarkObservation
  (Localizer& entity, const std::string& docstring)
    : Command (entity, boost::assign::list_of (Value::STRING), docstring)
  {}

  Value AddLandmarkObservation::doExecute ()
  {
    Localizer& localizer = static_cast<Localizer&> (owner ());
    const std::vector<Value>& values = getParameterValues();
    const std::string& landmarkName = values[0].value();

    LandmarkObservation* lmo =
      new LandmarkObservation (localizer, landmarkName);
    localizer.landmarkObservations_.push_back
      (boost::shared_ptr<LandmarkObservation> (lmo));
    return Value ();
  }
} // end of namespace command.
