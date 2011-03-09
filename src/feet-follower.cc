// Copyright 2010, François Bleibel, Thomas Moulard, Olivier Stasse,
// JRL, CNRS/AIST.
//
// This file is part of dynamic-graph.
// dynamic-graph is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// dynamic-graph is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// dynamic-graph. If not, see <http://www.gnu.org/licenses/>.

#include <string>
#include <fstream>

#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <jrl/mal/boost.hh>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command.h>
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/null-ptr.hh>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot-dynamic/dynamic.h>


#include "common.hh"

namespace ml = ::maal::boost;
namespace dg = ::dynamicgraph;
namespace sot = ::dynamicgraph::sot;

sot::MatrixHomogeneous
transformPgFrameIntoAnkleFrame (double tx, double ty, double tz, double theta,
				const sot::MatrixHomogeneous& feetToAnkle)
{
  ml::Vector t (3);
  t (0) = tx;
  t (1) = ty;
  t (2) = tz;

  sot::VectorRollPitchYaw vR;
  vR (2) = theta;
  sot::MatrixRotation R;
  vR.toMatrix (R);

  sot::MatrixHomogeneous tmp;
  tmp.buildFrom (R, t);
  sot::MatrixHomogeneous tmp2;
  tmp2 = tmp * feetToAnkle;
  return tmp2;
}

class FeetFollower;
namespace command
{
  using ::dynamicgraph::command::Command;
  using ::dynamicgraph::command::Value;

  class Start : public Command
  {
  public:
    Start (FeetFollower& entity, const std::string& docstring);
    virtual Value doExecute();
  };
}

using ::dynamicgraph::command::Setter;

class FeetFollower : public dg::Entity
{
 public:
  typedef dg::SignalTimeDependent<ml::Vector, int> signalCoM_t;
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signalFoot_t;

  explicit FeetFollower (const std::string& name)
    : Entity(name),
      t_ (),
      com_ (3),
      zmp_ (3),
      leftAnkle_ (),
      rightAnkle_ (),
      comZ_ (0.),
      leftFootToAnkle_ (),
      rightFootToAnkle_ (),
      initialLeftAnklePosition_ (),
      initialRightAnklePosition_ (),
      started_ (false),
      comOut_ (INIT_SIGNAL_OUT ("com", FeetFollower::updateCoM, "Vector")),
      zmpOut_ (INIT_SIGNAL_OUT ("zmp", FeetFollower::updateZmp, "Vector")),
      leftAnkleOut_
      (INIT_SIGNAL_OUT
       ("left-ankle", FeetFollower::updateLeftAnkle, "MatrixHomo")),
      rightAnkleOut_
      (INIT_SIGNAL_OUT
       ("right-ankle", FeetFollower::updateRightAnkle, "MatrixHomo"))
  {
    signalRegistration (zmpOut_ << comOut_ << leftAnkleOut_ << rightAnkleOut_);

    zmpOut_.setNeedUpdateFromAllChildren (true);
    comOut_.setNeedUpdateFromAllChildren (true);
    leftAnkleOut_.setNeedUpdateFromAllChildren (true);
    rightAnkleOut_.setNeedUpdateFromAllChildren (true);

    std::string docstring;
    addCommand ("setComZ", new Setter<FeetFollower, double>
		(*this, &FeetFollower::setComZ, docstring));

    addCommand ("setLeftFootToAnkle",
		new Setter<FeetFollower, maal::boost::Matrix>
		(*this, &FeetFollower::setLeftFootToAnkle, docstring));
    addCommand ("setRightFootToAnkle",
		new Setter<FeetFollower, maal::boost::Matrix>
		(*this, &FeetFollower::setRightFootToAnkle, docstring));

    addCommand ("setInitialLeftFootPosition",
		new Setter<FeetFollower, maal::boost::Matrix>
		(*this, &FeetFollower::setInitialLeftAnklePosition, docstring));
    addCommand ("setInitialRightFootPosition",
		new Setter<FeetFollower, maal::boost::Matrix>
		(*this, &FeetFollower::setInitialRightAnklePosition, docstring));

    addCommand ("start", new command::Start (*this, docstring));
  }

  virtual ~FeetFollower ()
  {}

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

  void start ()
  {
    started_ = true;
    impl_start ();
  }

protected:
  virtual void impl_update () = 0;
  virtual void impl_start () = 0;

  int t_;
  ml::Vector com_;
  ml::Vector zmp_;
  sot::MatrixHomogeneous leftAnkle_;
  sot::MatrixHomogeneous rightAnkle_;

  double comZ_;
  sot::MatrixHomogeneous leftFootToAnkle_;
  sot::MatrixHomogeneous rightFootToAnkle_;
  sot::MatrixHomogeneous initialLeftAnklePosition_;
  sot::MatrixHomogeneous initialRightAnklePosition_;

  bool started_;

private:
  void update (int t)
  {
     if (t <= t_)
       return;
    t_ = t;
    impl_update ();
  }

  ml::Vector& updateCoM (ml::Vector& res, int t)
  {
    if (t > t_)
      update (t);
    res = com_;
    return res;
  }

  ml::Vector& updateZmp (ml::Vector& res, int t)
  {
    if (t > t_)
      update (t);
    res = zmp_;
    return res;
  }

  sot::MatrixHomogeneous& updateLeftAnkle (sot::MatrixHomogeneous& res, int t)
  {
    if (t > t_)
      update (t);
    res = leftAnkle_;
    return res;
  }

  sot::MatrixHomogeneous& updateRightAnkle (sot::MatrixHomogeneous& res, int t)
  {
     if (t > t_)
      update (t);
    res = rightAnkle_;
    return res;
  }

  void setComZ(const double& v)
  {
    comZ_ = v;
  }

  void setLeftFootToAnkle(const maal::boost::Matrix& v)
  {
    leftFootToAnkle_ = v;
  }

  void setRightFootToAnkle(const maal::boost::Matrix& v)
  {
    rightFootToAnkle_ = v;
  }

  void setInitialLeftAnklePosition(const maal::boost::Matrix& v)
  {
    initialLeftAnklePosition_ = v;
  }

  void setInitialRightAnklePosition(const maal::boost::Matrix& v)
  {
    initialRightAnklePosition_ = v;
  }

  signalCoM_t comOut_;
  signalCoM_t zmpOut_;
  signalFoot_t leftAnkleOut_;
  signalFoot_t rightAnkleOut_;
};

namespace command {
  Start::Start (FeetFollower& entity, const std::string& docstring)
    : Command (entity, std::vector<Value::Type> (), docstring)
  {}

  Value Start::doExecute()
  {
    FeetFollower& entity = static_cast<FeetFollower&>(owner ());
    entity.start ();
    return Value ();
  }
} // end of namespace command.


class FeetFollowerFromFile : public FeetFollower
{
public:
  static const std::string CLASS_NAME;

  explicit FeetFollowerFromFile (const std::string& name)
    : FeetFollower (name),
      trajectory_ (),
      index_ (0)
  {
  }

  void readTrajectory ()
  {
    std::ifstream leftAnkleTrajFile ("left-ankle.dat");
    std::ifstream rightAnkleTrajFile ("right-ankle.dat");
    std::ifstream comTrajFile ("com.dat");
    std::ifstream zmpTrajFile ("zmp.dat");

    while (leftAnkleTrajFile.good ()
	   && rightAnkleTrajFile.good ()
	   && comTrajFile.good ()
	   && zmpTrajFile.good ())
      {
	// X Y Z Theta?
	double left[4];
	double right[4];

	// X Y (Z = 0)
	ml::Vector com (3);
	ml::Vector zmp (3);

	leftAnkleTrajFile >> left[0] >> left[1] >> left[2] >> left[3];
	rightAnkleTrajFile >> right[0] >> right[1] >> right[2] >> right[3];
	comTrajFile >> com (0) >> com (1);
	zmpTrajFile >> zmp (0) >> zmp (1);

	com (2) = comZ_; // comZ is fixed.
	zmp (2) = 0.; // ZMP is in global frame.

	sot::MatrixHomogeneous leftAnkle =
	  transformPgFrameIntoAnkleFrame (left[0], left[1], left[2], left[3],
					  leftFootToAnkle_);
	sot::MatrixHomogeneous rightAnkle =
	  transformPgFrameIntoAnkleFrame (right[0], right[1],
					  right[2], right[3],
					  rightFootToAnkle_);

	trajectory_.leftAnkle.push_back (leftAnkle);
	trajectory_.rightAnkle.push_back (rightAnkle);
	trajectory_.com.push_back (com);
	trajectory_.zmp.push_back (zmp);
      }

    if (leftAnkleTrajFile.good ()
	|| rightAnkleTrajFile.good ()
	|| comTrajFile.good ()
	|| zmpTrajFile.good ())
      {
	std::cerr << "WARNING: trajectory size does not match." << std::endl;
	return;
      }
  }

  ~FeetFollowerFromFile ()
  {}

private:
  virtual void impl_update ()
  {
    if (trajectory_.com[index_].size () != 3
	|| trajectory_.zmp[index_].size () != 3)
      {
	std::cerr << "bad size" << std::endl;
      }

    leftAnkle_ = trajectory_.leftAnkle[index_];
    rightAnkle_ = trajectory_.rightAnkle[index_];
    com_ = trajectory_.com[index_];
    zmp_ = trajectory_.zmp[index_];

    if (started_ && index_ < trajectory_.leftAnkle.size () - 1)
      ++index_;
  }

  virtual void impl_start ()
  {
    readTrajectory ();
  }

private:
  struct Trajectory
  {
    std::vector<sot::MatrixHomogeneous> leftAnkle;
    std::vector<sot::MatrixHomogeneous> rightAnkle;
    std::vector<ml::Vector> com;
    std::vector<ml::Vector> zmp;
  };

  Trajectory trajectory_;
  unsigned index_;
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeetFollowerFromFile,
				   "FeetFollowerFromFile");


class PostureError : public dg::Entity
{
 public:
  typedef dg::SignalPtr<ml::Vector, int> signalIn_t;
  typedef dg::SignalTimeDependent<ml::Vector, int> signalOut_t;

  static const std::string CLASS_NAME;

  explicit PostureError (const std::string& name)
    : Entity(name),
      state_
      (dg::nullptr,
       MAKE_SIGNAL_STRING (name, true, "Vector", "state")),
      error_ (INIT_SIGNAL_OUT ("error", PostureError::updateError, "Vector"))
  {
    signalRegistration (error_ << state_);

    error_.addDependency (state_);
  }

  virtual ~PostureError ()
  {}

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

private:
  ml::Vector& updateError (ml::Vector& res, int t)
  {
    ml::Vector state = state_ (t);

    int errorSize = state.size () - 12 - 3;
    if (errorSize < 0)
      return res;

    res.resize (errorSize);

    res (0) = state (3);
    res (1) = state (4);
    res (2) = state (5);

    for (unsigned i = 0; i < errorSize - 3u; ++i)
      res (i + 3) = state (i + 6 + 12);
    return res;
  }

  signalIn_t state_;
  signalOut_t error_;
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PostureError, "PostureError");
