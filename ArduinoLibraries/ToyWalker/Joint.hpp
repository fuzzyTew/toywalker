#pragma once

#include "ServoImplementation.hpp"

class Joint
{
public:
	Joint(ServoImplementation * servo, double radiansA, double radiansB);

	void setFromA(double radians);
	void setFromB(double radians);

	double range() { return signFromA * (radiansB - radiansA); }

	ServoImplementation & getServo() { return *servo; }

private:
	ServoImplementation * servo;
	double radiansA;
	double radiansB;
	double signFromA;
};
