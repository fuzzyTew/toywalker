#include "Joint.hpp"

double signum(double f) { return (0 < f) - (f < 0); }

Joint::Joint(ServoImplementation * servo, double radiansA, double radiansB)
: servo(servo),
  radiansA(radiansA),
  radiansB(radiansB),
  signFromA(signum(radiansB - radiansA))
{ }

void Joint::setFromA(double radians)
{
	servo->go(radiansA + signFromA * radians);
}

void Joint::setFromB(double radians)
{
	servo->go(radiansB - signFromA * radians);
}
