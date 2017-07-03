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
	LegSymmetric(Joint aimingHip, Joint liftingHip, Joint knee, float kneeMin);

	void setLift(float radians);
	void setAimFromA(float radians) { aimingHip.setFromA(radians); }
	void setAimFromB(float radians) { aimingHip.setFromB(radians); }
	void setKnee(float radians);

	float kneeMin() { return kneeContractionRadians; }
	float liftRange() { return liftingHip.range(); }
	float aimRange() { return aimingHip.range(); }

	Joint & getAim() { return aimingHip; }
	Joint & getLift() { return liftingHip; }
	Joint & getKnee() { return knee; }

private:
	void updateLift();

	Joint aimingHip;
	Joint liftingHip;
	Joint knee;

	float kneeContractionRadians;

	float liftRadians;
	float kneeOffset;
};
