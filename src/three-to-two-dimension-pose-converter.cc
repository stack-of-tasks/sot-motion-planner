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

#include "common.hh"
#include "three-to-two-dimension-pose-converter.hh"

ThreeToTwoDimensionPoseConverter::ThreeToTwoDimensionPoseConverter
(const std::string& name)
  : dg::Entity (name),
    in_ (dg::nullptr,
	 MAKE_SIGNAL_STRING (name, true, "MatrixHomo", "in")),
    out_ (INIT_SIGNAL_OUT
	  ("out", ThreeToTwoDimensionPoseConverter::update, "Vector"))
{
  signalRegistration (in_ << out_);
  out_.setNeedUpdateFromAllChildren (true);
}

ThreeToTwoDimensionPoseConverter::~ThreeToTwoDimensionPoseConverter()
{}

ml::Vector&
ThreeToTwoDimensionPoseConverter::update (ml::Vector& res, int t)
{
  res = MatrixHomogeneousToXYTheta(in_.access (t));
  return res;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (ThreeToTwoDimensionPoseConverter,
				    "ThreeToTwoDimensionPoseConverter");
