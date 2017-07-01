#include "LegSymmetric.hpp"

#include <math.h>

LegSymmetric::LegSymmetric(Joint aimingHip, Joint liftingHip, Joint knee, double kneeMinRadians)
: aimingHip(aimingHip),
  liftingHip(liftingHip),
  knee(knee),
  kneeContractionRadians(kneeMinRadians),
  liftRadians(liftingHip.range() / 2.0)
{
	setKnee(M_PI_2);
}

void LegSymmetric::setLift(double radians)
{
	liftRadians = radians;

	updateLift();
}

void LegSymmetric::setKnee(double radians)
{
	knee.setFromA(radians - kneeContractionRadians);

	kneeOffset = (M_PI - radians) / 2.0;

	updateLift();
}

void LegSymmetric::updateLift()
{
	double limited = fminf(liftRadians + kneeOffset, liftingHip.range());

	liftingHip.setFromA(limited);
}
