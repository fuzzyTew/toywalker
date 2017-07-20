#pragma once

#include <Servo.h>

#include "Vector.hpp"

namespace toywalker {

template <int pin, int minMicros, int maxMicros, long long extentMillidegrees>
class ServoPermanentArduino
{
public:
	ServoPermanentArduino()
	: limits{ ANGLE_LIMIT_MIN, ANGLE_LIMIT_MAX }
	{
		activate();
		go(0);
	}

	void activate()
	{
		servo.attach(pin, minMicros, maxMicros);
	}

	bool activated()
	{
		return servo.attached();
	}

	void deactivate()
	{
		servo.detach();
	}

	int id() const
	{
		return pin;
	}

	Real angleGoal(Real radians)
	{
		int micros = radians * (180000.0 * (maxMicros - minMicros) / extentMillidegrees / M_PI);
		servo.writeMicroseconds(micros + (maxMicros + minMicros) / 2);
		return _radians = micros * (M_PI * extentMillidegrees / (maxMicros - minMicros) / 180000);
	}

	Real angleGoal() const
	{
		return _radians;
	}

	Array2 angleLimit() const
	{
		return limits;
	}

	Array2 angleLimit(Array2 const & radians)
	{
		limits = radians;
	}

	Array2 angleLimitMax()
	{
		return { ANGLE_LIMIT_MIN, ANGLE_LIMIT_MAX };
	}

	static constexpr Real ANGLE_LIMIT_MIN = -extentMillidegrees * M_PI / 180000 / 2;
	static constexpr Real ANGLE_LIMIT_MAX = extentMillidegrees * M_PI / 180000 / 2;

private:
	Servo servo;
	Array2 limits;
};

}
