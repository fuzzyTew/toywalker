#pragma once

#include "Walker.hpp"
#include "Geometry.hpp"

namespace toywalker {

// Wraps the concept of when to pick up and drop which legs
class WalkGait {
public:
	void start(Walker & walker);

	virtual void step(Real pathSeconds, Limb *& limb, Real & secondsInAir, Real & secondsToThisFootAgain, Real & secondsToAnyFoot) = 0;

	virtual void stop() { }

	Walker & walker() { return *_walker; }
	Path<Isometry3> & bodyPath() { return walker().path(); }

protected:
	virtual void start() { }

private:
	Walker * _walker;
};

}

// impl

namespace toywalker {

void WalkGait::start(Walker & walker)
{
	_walker = &walker;
	start();
}

}
