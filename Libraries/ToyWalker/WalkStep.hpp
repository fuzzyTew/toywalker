#pragma once

#include "Walker.hpp"
#include "Vector.hpp"

namespace toywalker {

// Actually lifts and moves a leg, deciding how high to lift it and what path
// to move it through
class WalkStep
{
public:
	WalkStep(Walker & walker, Limb & limb);
	~WalkStep();

	void start(Vector3 destinationArea, Real secondsInAir);

	virtual Limb::Angles at(Real time) = 0;

	virtual void stop() { }

	Walker & walker() { return _walker; }
	Limb & limb() { return _limb; }
	Path<Isometry3> & bodyPath() { return walker().path(); }
	Vector3 const & originArea() { return _originArea; }
	Vector3 const & destinationArea() { return _destinationArea; }
	Real duration() { return _duration; }

protected:
	virtual void start() { }

private:
	Walker & _walker;
	Limb & _limb;
	Path<Isometry3> * _path;
	Vector3 _originArea, _destinationArea;
	Real _duration;
};

}

// impl

namespace toywalker {

WalkStep::WalkStep(Walker & walker, Limb & limb)
: _walker(walker), _limb(limb)
{
	this->limb().setStep(this);
}

WalkStep::~WalkStep()
{
	limb().removeStep();
}

void WalkStep::start(Vector3 destinationArea, Real secondsInAir)
{
	_originArea = limb().footGoalArea();
	_destinationArea = destinationArea;
	_duration = secondsInAir;

	start();
}

}
