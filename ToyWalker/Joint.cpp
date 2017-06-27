#include "Joint.hpp"

float signum(float f) { return (0 < f) - (f < 0); }

Joint::Joint(ServoImplementation * servo, float radiansA, float radiansB)
: servo(servo),
  radiansA(radiansA),
  radiansB(radiansB),
  signFromA(signum(radiansB - radiansA))
{ }

void Joint::setFromA(float radians)
{
	servo->go(radiansA + signFromA * radians);
}

void Joint::setFromB(float radians)
{
	servo->go(radiansB - signFromA * radians);
}
