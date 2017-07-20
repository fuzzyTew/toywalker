#pragma once

#include "ToyWalker.h"

#include "Vector.hpp"

namespace toywalker {

class Servo
{
public:

	virtual void activate() { };
	virtual bool activated() { return true; }
	virtual void deactivate() { };

	virtual bool hasPresent() { return false; }
	virtual bool present() { return true; }

	virtual size_t id() = 0;

	virtual Real angleGoal() = 0;
	virtual Real angleGoal(Real radians) = 0;

	bool hasAngle() { return false; }
	virtual Real angle() { return angleGoal(); }

	virtual Array2 angleLimit() = 0;
	virtual Array2 angleLimit(Array2 const & radians) = 0;
	virtual Array2 angleLimitMax() = 0;

	bool hasVelocity() { return false; }
	virtual Real velocity() { return 0; }
	virtual Real velocityMax() = 0;

	bool hasVelocityGoal() { return false; }
	virtual Real velocityGoal() { return 0; }
	virtual Real velocityGoal(Real radiansPerSecond) { return 0; }

	bool hasTorque() { return false; }
	virtual Real torque() { return 0; }
	virtual Real torqueMax() = 0;

	bool hasTorqueLimit() { return false; }
	virtual Real torqueLimit() { return 0; }
	virtual Real torqueLimit(Real newtonDecimeters) { return 0; }

	virtual Array3 dynamics() { return {angle(), velocity(), torque()}; }

	bool hasAlert() { return false; }
	virtual void alert(bool alert = true) { if(alert) angleGoal(0); }
};

}
