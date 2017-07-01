#pragma once

#include "Joint.hpp"

/**
 * A leg where both calf and thigh are the same length.
 */
class LegSymmetric
{
public:
	/**
	 * The lifting hip should be passed such that A lowers the leg and B raises it.
	 * The knee should be passed such that A is contraction and B is extension.
	 */
	LegSymmetric(Joint aimingHip, Joint liftingHip, Joint knee, double kneeMin);

	void setLift(double radians);
	void setAimFromA(double radians) { aimingHip.setFromA(radians); }
	void setAimFromB(double radians) { aimingHip.setFromB(radians); }
	void setKnee(double radians);

	double kneeMin() { return kneeContractionRadians; }
	double liftRange() { return liftingHip.range(); }
	double aimRange() { return aimingHip.range(); }

	Joint & getAim() { return aimingHip; }
	Joint & getLift() { return liftingHip; }
	Joint & getKnee() { return knee; }

private:
	void updateLift();

	Joint aimingHip;
	Joint liftingHip;
	Joint knee;

	double kneeContractionRadians;

	double liftRadians;
	double kneeOffset;
};
