#pragma once

#include "ToyWalker.h"

#include "WalkStep.hpp"

#include "Limb.hpp"
#include "PathSegmentQuadratic.hpp"

namespace toywalker {

// Calculates steps with minimal IK.  IK is used to find the step peak, and then the servo angles are interpolated in jointspace.
class WalkStepCompromise : public WalkStep
{
public:
	WalkStepCompromise(Walker & walker, Limb & limb, Real stepHeight);

	Real stepHeight() { return _stepHeight; }
	void stepHeight(Real height) { _stepHeight = height; }

	virtual void start();
	
	virtual Limb::Angles at(Real seconds);

private:
	Real _stepHeight;

	PathSegmentQuadratic<Limb::Angles> liftPath;
};

}

// impl

namespace toywalker {

WalkStepCompromise::WalkStepCompromise(Walker & walker, Limb & limb, Real stepHeight)
: WalkStep(walker, limb),
  _stepHeight(stepHeight),
  liftPath(Limb::Angles(limb.servos()), Limb::Angles(limb.servos()), 0.5, Limb::Angles(limb.servos()), 1)
{ }

void WalkStepCompromise::start()
{
	Limb::Angles beginning = limb().anglesGoal();

	Limb::Angles end = limb().planBody(bodyPath().at(bodyPath().now() + duration()).inverse() * destinationArea());

	Vector3 middleArea = (originArea() + destinationArea()) / 2;
	middleArea.z() += stepHeight();
	Limb::Angles middle = limb().planBody(bodyPath().at(bodyPath().now() + duration()/2).inverse() * middleArea);

	liftPath.constraints(beginning, middle, duration()/2, end, duration());
}

Limb::Angles WalkStepCompromise::at(Real seconds)
{
	return liftPath.at(seconds);
}

}
