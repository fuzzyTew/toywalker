#pragma once

#include "ToyWalker.h"

#include "WalkGait.hpp"

namespace toywalker {

class WalkGaitFormula : public WalkGait {
public:
	struct Step
	{
		size_t limb;
		Real liftSeconds;
		Real dropSeconds;
	};

	WalkGaitFormula(Real period, std::initializer_list<Step> steps)
	: _period(period),
	  _steps(decltype(_steps)::Map(steps.begin(), steps.size())),
	  _step(0)
	{ }

	virtual void step(Real pathSeconds, Limb *& limb, Real & secondsInAir, Real & secondsToThisFootAgain, Real & secondsToAnyFoot)
	{
		Real time = _steps[_step].liftSeconds;
		limb = &walker().limb(_steps[_step].limb);
		secondsInAir = _steps[_step].dropSeconds - _steps[_step].liftSeconds;
		secondsToThisFootAgain = _period;

		++ _step;

		if (_step < _steps.size()) {
			secondsToAnyFoot = _steps[_step].liftSeconds - time;
		} else {
			_step = 0;
			secondsToAnyFoot = (_period - time) + _steps[_step].liftSeconds;
		}
	}

private:
	Real _period;
	Eigen::Array<Step, Eigen::Dynamic, 1, 0, Body::MAX_LIMBS, 1> _steps;

	int _step;
};

}
