#pragma once

#include "ToyWalker.h"

#include <Eigen/Core>

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

	virtual double angleGoal() = 0;
	virtual double angleGoal(double radians) = 0;

	bool hasAngle() { return false; }
	virtual double angle() { return angleGoal(); }

	virtual Eigen::Array2d angleLimit() = 0;
	virtual Eigen::Array2d angleLimit(Eigen::Array2d const & radians) = 0;
	virtual Eigen::Array2d angleLimitMax() = 0;

	bool hasVelocity() { return false; }
	virtual double velocity() { return 0; }
	virtual double velocityMax() = 0;

	bool hasVelocityGoal() { return false; }
	virtual double velocityGoal() { return 0; }
	virtual double velocityGoal(double radiansPerSecond) { return 0; }

	bool hasTorque() { return false; }
	virtual double torque() { return 0; }
	virtual double torqueMax() = 0;

	bool hasTorqueLimit() { return false; }
	virtual double torqueLimit() { return 0; }
	virtual double torqueLimit(double newtonDecimeters) { return 0; }

	virtual Eigen::Array3d dynamics() { return {angle(), velocity(), torque()}; }
};

}
