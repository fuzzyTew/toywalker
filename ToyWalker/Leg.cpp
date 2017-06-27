#include "Leg.hpp"

#include <math.h>

Leg::Leg(Joint aimingHip, Joint liftingHip, Joint knee, float kneeMinRadians)
: aimingHip(aimingHip),
  liftingHip(liftingHip),
  knee(knee),
  kneeContractionRadians(kneeMinRadians),
  liftRadians(liftingHip.range() / 2.0)
{
	setKnee(M_PI_2);
}

void Leg::setLift(float radians)
{
	liftRadians = radians;

	updateLift();
}

void Leg::setKnee(float radians)
{
	knee.setFromA(radians - kneeContractionRadians);

	kneeOffset = (M_PI - radians) / 2.0;

	updateLift();
}

void Leg::updateLift()
{
	float limited = fminf(liftRadians + kneeOffset, liftingHip.range());

	liftingHip.setFromA(limited);
}
