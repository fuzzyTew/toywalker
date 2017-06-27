#pragma once

#include "ServoImplementation.hpp"

class Joint
{
public:
	Joint(ServoImplementation * servo, float radiansA, float radiansB);

	void setFromA(float radians);
	void setFromB(float radians);

	float range() { return signFromA * (radiansB - radiansA); }

	ServoImplementation & getServo() { return *servo; }

private:
	ServoImplementation * servo;
	float radiansA;
	float radiansB;
	float signFromA;
};
