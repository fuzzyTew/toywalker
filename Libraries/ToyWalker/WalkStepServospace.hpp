#pragma once

#include "ToyWalker.h"

#include "WalkStep.hpp"

#include "Limb.hpp"
#include "PathSegment.hpp"

namespace toywalker {

// Useful for concise path generation
Limb::Angles limbAngles(std::initializer_list<Real> angles);

// Takes a step adjusting only the lifting servos and interpolating the others, without any IK
class WalkStepServospace : public WalkStep
{
public:
	WalkStepServospace(Walker & walker, Limb & limb, std::initializer_list<size_t> liftServos, PathSegment<Limb::Angles> & liftPaths);

	virtual void start();

	virtual Limb::Angles at(Real seconds);

private:
	ArrayX<Limb::MAX_JOINTS, size_t> liftServos;
	ArrayX<Limb::MAX_JOINTS, int> pathIndices;
	PathSegment<Limb::Angles> & liftPaths;

	Limb::Angles beginning, end;
};


}

// impl

namespace toywalker {

Limb::Angles limbAngles(std::initializer_list<Real> angles)
{
	return Limb::Angles::Map(angles.begin(), angles.size());
}

WalkStepServospace::WalkStepServospace(Walker & walker, Limb & limb, std::initializer_list<size_t> liftServos, PathSegment<Limb::Angles> & liftPaths)
: WalkStep(walker, limb),
  liftServos(decltype(this->liftServos)::Map(liftServos.begin(), liftServos.size())),
  liftPaths(liftPaths)
{ } 

void WalkStepServospace::start()
{
	beginning = limb().anglesGoal();
	end = limb().planBody(bodyPath().at(bodyPath().now() + duration()).inverse() * destinationArea());

	Limb::Angles pathBeginning(liftServos.size()), pathEnd(liftServos.size());

	pathIndices.resize(limb().servos());
	pathIndices.fill(-1);

	for (size_t i = 0; i < liftServos.size(); ++ i) {
		pathBeginning[i] = beginning[liftServos[i]];
		pathEnd[i] = end[liftServos[i]];

		pathIndices[liftServos[i]] = i;
	}

	liftPaths.constraints(pathBeginning, pathEnd, duration());
}

Limb::Angles WalkStepServospace::at(Real seconds)
{
	Limb::Angles pathPositions = liftPaths.at(seconds);
	Limb::Angles angles(limb().servos());

	Real pct = seconds / duration();
	for (size_t i = 0; i < limb().servos(); ++ i) {
		int pathIndex = pathIndices[i];
		if (pathIndex == -1) {
			// linear interpolation
			angles[i] = (end[i] - beginning[i]) * pct + beginning[i];
		} else {
			// lifting path
			angles[i] = pathPositions[pathIndex];
		}
	}

	return angles;
}

}
