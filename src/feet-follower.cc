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
#include <dynamic-graph/entity.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/pool.h>
#include <dynamic-graph/null-ptr.hh>
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

ml::Vector
transformPgFrameIntoAnkleFrameCom (double comx, double comy, double comz,
				   const sot::MatrixHomogeneous& feetToAnkle)
{
  sot::MatrixHomogeneous tmp;
  tmp (0, 3) = comx, tmp (1, 3) = comy, tmp (2, 3) = comz;
  //FIXME: we suppose feetToAnkle is the same for both feet.

  sot::MatrixHomogeneous tmp2;
  tmp2 = tmp * feetToAnkle;

  ml::Vector res (3);
  for (unsigned i = 0; i < 3; ++i)
    res (i) =  tmp2 (i, 3);
  return res;
}


class FeetFollower : public dg::Entity
{
 public:
  typedef dg::SignalTimeDependent<ml::Vector, int> signalCoM_t;
  typedef dg::SignalTimeDependent<sot::MatrixHomogeneous, int> signalFoot_t;

  explicit FeetFollower (const std::string& name)
    : Entity(name),
      t_ (),
      com_ (3),
      leftAnkle_ (),
      rightAnkle_ (),
      comOut_ (INIT_SIGNAL_OUT ("com", FeetFollower::updateCoM, "Vector")),
      leftAnkleOut_
      (INIT_SIGNAL_OUT
       ("left-ankle", FeetFollower::updateLeftAnkle, "MatrixHomo")),
      rightAnkleOut_
      (INIT_SIGNAL_OUT
       ("right-ankle", FeetFollower::updateRightAnkle, "MatrixHomo"))
  {
    signalRegistration (comOut_ << leftAnkleOut_ << rightAnkleOut_);

    comOut_.setNeedUpdateFromAllChildren (true);
    leftAnkleOut_.setNeedUpdateFromAllChildren (true);
    rightAnkleOut_.setNeedUpdateFromAllChildren (true);
  }

  virtual ~FeetFollower ()
  {}

  virtual const std::string& getClassName ()
  {
    return CLASS_NAME;
  }

protected:
  virtual void impl_update () = 0;

  int t_;
  ml::Vector com_;
  sot::MatrixHomogeneous leftAnkle_;
  sot::MatrixHomogeneous rightAnkle_;

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

  signalCoM_t comOut_;
  signalFoot_t leftAnkleOut_;
  signalFoot_t rightAnkleOut_;
};

class FeetFollowerFromFile : public FeetFollower
{
public:
  static const std::string CLASS_NAME;

  explicit FeetFollowerFromFile (const std::string& name)
    : FeetFollower (name),
      leftAnkleTrajFile_ ("left-ankle.dat"),
      rightAnkleTrajFile_ ("right-ankle.dat"),
      comTrajFile_ ("com.dat"),
      feetToAnkleLeft_ (dg::nullptr,
			MAKE_SIGNAL_STRING
			(name, true, "MatrixHomo", "feetToAnkleLeft")),
      feetToAnkleRight_ (dg::nullptr,
			 MAKE_SIGNAL_STRING
			 (name, true, "MatrixHomo", "feetToAnkleRight"))
  {
    signalRegistration (feetToAnkleLeft_ << feetToAnkleRight_);
  }

  ~FeetFollowerFromFile ()
  {}

private:
  virtual void impl_update ()
  {
    if (!leftAnkleTrajFile_.good ())
      {
	std::cerr << "bad left ankle" << std::endl;
	return;
      }
    if (!rightAnkleTrajFile_.good ())
      {
	std::cerr << "bad right ankle" << std::endl;
	return;
      }
    if (!comTrajFile_.good ())
      {
	sd::cerr << "bad com" << std::endl;
	return;
      }

    // X Y Z Theta?
    double left[4];
    double right[4];

    // X Y (Z = 0)
    double com[2];

    leftAnkleTrajFile_ >> left[0] >> left[1] >> left[2] >> left[3];
    rightAnkleTrajFile_ >> right[0] >> right[1] >> right[2] >> right[3];
    comTrajFile_ >> com[0] >> com[1];

    //FIXME: check that (comz = 0.7?).
    com_ = transformPgFrameIntoAnkleFrameCom (com[0], com[1], 0.7,
					      feetToAnkleLeft_ (t_));
    leftAnkle_ =
      transformPgFrameIntoAnkleFrame (left[0], left[1], left[2], left[3],
				      feetToAnkleLeft_ (t_));
    rightAnkle_ =
      transformPgFrameIntoAnkleFrame (right[0], right[1], right[2], right[3],
				      feetToAnkleRight_ (t_));
  }

private:
  std::ifstream leftAnkleTrajFile_;
  std::ifstream rightAnkleTrajFile_;
  std::ifstream comTrajFile_;

  typedef dg::SignalPtr<sot::MatrixHomogeneous,int> signalInMatrix_t;
  signalInMatrix_t feetToAnkleLeft_;
  signalInMatrix_t feetToAnkleRight_;
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeetFollowerFromFile, "FeetFollowerFromFile");


#ifdef DISABLED_FOR_NOW

class FeetFollowerOnline : public FeetFollower
{
public:
  static const std::string CLASS_NAME;

  explicit FeetFollowerOnline (const std::string& name)
    : FeetFollower (name),
      i_ (0),
      patternGenerator_ (),
      steps_ (),
      feetToAnkleLeft_ (dg::nullptr,
			MAKE_SIGNAL_STRING
			(name, true, "MatrixHomo", "feetToAnkleLeft")),
      feetToAnkleRight_ (dg::nullptr,
			 MAKE_SIGNAL_STRING
			 (name, true, "MatrixHomo", "feetToAnkleRight"))
  {
    signalRegistration (feetToAnkleLeft_ << feetToAnkleRight_);

    double body_height = 0.814;
    char left_or_right = 'L';
    double t1 = 0.95;
    double t2 = 1.05;
    double t3 = 2.0;
    double incrTime = 0.005;
    double g  = 9.81;
    std::vector<double> vect_inputs;

    patternGenerator_.produceSeqSlidedHalfStepFeatures
      (steps_, incrTime, body_height, g, t1, t2, t3, vect_inputs, left_or_right);
  }

  ~FeetFollowerOnline ()
  {}

private:
  virtual void impl_update ()
  {
    //FIXME: check that (comz = 0.7?).
    com_ = transformPgFrameIntoAnkleFrameCom
      (steps_.comTrajX[i_], steps_.comTrajY[i_], 0.7, feetToAnkleLeft_ (t_));

    leftAnkle_ =
      transformPgFrameIntoAnkleFrame
      (steps_.leftfootXtraj[i_], steps_.leftfootYtraj[i_],
       steps_.leftfootHeight[i_], steps_.leftfootOrient[i_],
       feetToAnkleLeft_ (t_));

    rightAnkle_ =
      transformPgFrameIntoAnkleFrame
      (steps_.rightfootXtraj[i_], steps_.rightfootYtraj[i_],
       steps_.rightfootHeight[i_], steps_.rightfootOrient[i_],
       feetToAnkleRight_ (t_));

    // impl_update is only called *once* per time increment, so we
    // are sure next call will be at t+0.05.
    i_++;
  }

private:
  unsigned i_;
  CnewPGstepStudy patternGenerator_;
  StepFeatures steps_;

  typedef dg::SignalPtr<sot::MatrixHomogeneous,int> signalInMatrix_t;
  signalInMatrix_t feetToAnkleLeft_;
  signalInMatrix_t feetToAnkleRight_;
};

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeetFollowerOnline, "FeetFollowerOnline");

#endif //! DISABLED_FOR_NOW
