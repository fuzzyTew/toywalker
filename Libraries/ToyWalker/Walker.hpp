#pragma once

#include "Body.hpp"

#include "Path.hpp"

namespace toywalker {

class Ground;
class WalkGait;
class WalkStep;

class Walker : public Body
{
public:
	Walker(Vector3 centerOfMass, Ground & ground, WalkGait & gait, Real minStepDistance = 0);

	Real heightPct(Real pct, Isometry3 const & bodyToArea);
	Array2 heightRange(Isometry3 const & bodyToArea);

	ArrayX<MAX_LIMBS, Vector3> attachments();
	Vector3 bodyHomeArea();
	Vector3 bodyHomeArea(ArrayX<MAX_LIMBS, Vector3> const & attachments);

	Ground & ground() { return *_ground; }
	void ground(Ground & ground) { _ground = &ground; }

	Path<Isometry3> & path() { return *_path; }
	void path(Path<Isometry3> & path);

	WalkGait & gait() { return *_gait; }
	void gait(WalkGait & gait);

	Real minStepDistance() { return sqrt(minStepDistanceSquared); }
	void minStepDistance(Real decimeters) { minStepDistanceSquared = decimeters * decimeters; }

	void tick();

protected:
	virtual void addLimb(Limb * limb);

private:
	Vector3 _centerOfMass;
	Ground * _ground;
	Path<Isometry3> * _path;
	WalkGait * _gait;
	Real minStepDistanceSquared;

	Real nextStepSeconds;
	ArrayX<MAX_LIMBS, std::pair<WalkStep*, Real>> activeSteps;
};

}

// impl

#include "Ground.hpp"
#include "WalkGait.hpp"
#include "WalkStep.hpp"

namespace toywalker {

Walker::Walker(Vector3 centerOfMass, Ground & ground, WalkGait & gait, Real minStepDistance)
: _centerOfMass(centerOfMass),
  _ground(&ground),
  _path(nullptr),
  _gait(&gait),
  minStepDistanceSquared(minStepDistance*minStepDistance),
  nextStepSeconds(0),
  activeSteps((int)0)
{ }

Real Walker::heightPct(Real pct, Isometry3 const & orientation)
{
	Array2 range = heightRange(orientation);
	return (range[1] - range[0]) * pct + range[0];
}

Array2 Walker::heightRange(Isometry3 const & bodyToArea)
{
	// TODO: I think this function returns wrong values when the body tilts
	Vector3 bodySky = bodyToArea.inverse().linear() * Vector3::UnitZ();
	Array2 ret = {0, INFINITY};
	for (size_t i = 0; i < limbs(); ++ i) {
		auto reach = limb(i).reach();
		Real offset = bodySky.dot(limb(i).homeBody());
		if (reach[0] - offset > ret[0])
			ret[0] = reach[0] - offset;
		if (reach[1] - offset < ret[1])
			ret[1] = reach[1] - offset;
	}
	return ret;
}

ArrayX<Body::MAX_LIMBS, Vector3> Walker::attachments()
{
	ArrayX<MAX_LIMBS, Vector3> ret((int)0);
	for (size_t i = 0; i < limbs(); ++ i)
		if (limb(i).attached()) {
			ret.resize(ret.size() + 1);
			ret[ret.size() - 1] = limb(i).footGoalArea();
		}
	return ret;
}

Vector3 Walker::bodyHomeArea()
{
	return bodyHomeArea(attachments());
}

Vector3 Walker::bodyHomeArea(ArrayX<MAX_LIMBS, Vector3> const & attachments)
{
	Vector3 ret(0,0,0);
	size_t count = 0;
	for (int i = 0; i < attachments.size(); ++ i) {
		++ count;
		ret += attachments(i);
	}
	ret /= count;
	//ret.z() = height();
	return ret;
}

void Walker::path(Path<Isometry3> & path)
{
	if (_path) {
		_path->stop();
		_gait->stop();
	}
	_path = &path;
	nextStepSeconds = 0;
	_path->start();
	_gait->start(*this);
}

void Walker::gait(WalkGait & gait)
{
	_gait->stop();
	_gait = &gait;
	nextStepSeconds = 0;
	_gait->start(*this);
}

void Walker::tick()
{
	Real time = path().now();

	// Move along our path
	bodyToArea(path().at(time));

	// pick up any feet needed for this gait
	while (time >= nextStepSeconds) {
		Limb * sLimb;
		Real sInAir, sFootAgain, sAnyFoot;
		gait().step(time, sLimb, sInAir, sFootAgain, sAnyFoot);
		if (!sLimb->hasStep()) {
			sLimb->detach();
			sLimb->alert(true);
		} else {
			size_t idx = activeSteps.size();

			// plan the foot such that it will be in its home position in the middle of its time on the ground
			Vector3 destinationArea = ground().projection(path().at(time + (sInAir + sFootAgain) / 2) * sLimb->homeBody());
			if ((destinationArea - sLimb->footGoalArea()).squaredNorm() >= minStepDistanceSquared) {
				sLimb->detach();
				sLimb->step().start(destinationArea, sInAir);

				activeSteps.resize(idx + 1);
				activeSteps[idx].first = &sLimb->step();
				activeSteps[idx].second = time;
			} else {
				if (sAnyFoot > 0 && activeSteps.size() == 0)
				       sAnyFoot = 1.0 / 1024.0;	
			}
		}
		nextStepSeconds = time + sAnyFoot;
	}

	// move any feet that are in the air, stepping
	for (int i = 0; i < activeSteps.size(); ++ i) {
		WalkStep & step = *activeSteps[i].first;
		Real start = activeSteps[i].second;

		Real stepTime = time - start;
		if (stepTime < step.duration()) {
			Limb::Angles angles = step.at(time - start);
			step.limb().anglesGoal(angles);
		} else {
			step.limb().goalArea(step.destinationArea());
			step.limb().attach();
			for (int j = i; j < activeSteps.size() - 1; ++ j)
				activeSteps[j] = activeSteps[j + 1];
			activeSteps.resize(activeSteps.size() - 1);
			-- i;
		}
	}
}

void Walker::addLimb(Limb * limb)
{
	Body::addLimb(limb);

	limb->attach();
	limb->goalArea(ground().projection(bodyToArea() * limb->homeBody()));
}

}
