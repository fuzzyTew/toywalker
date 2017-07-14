#pragma once

#include <Servo.h>

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

	double angleGoal(double radians)
	{
		int micros = radians * (180000.0 * (maxMicros - minMicros) / extentMillidegrees / M_PI);
		servo.writeMicroseconds(micros + (maxMicros + minMicros) / 2);
		return _radians = micros * (M_PI * extentMillidegrees / (maxMicros - minMicros) / 180000);
	}

	double angleGoal() const
	{
		return _radians;
	}

	Eigen::Array2d angleLimit() const
	{
		return limits;
	}

	Eigen::Array2d angleLimit(Eigen::Array2d const & radians)
	{
		limits = radians;
	}

	Eigen::Array2d angleLimitMax()
	{
		return { ANGLE_LIMIT_MIN, ANGLE_LIMIT_MAX };
	}

	static constexpr double ANGLE_LIMIT_MIN = -extentMillidegrees * M_PI / 180000 / 2;
	static constexpr double ANGLE_LIMIT_MAX = extentMillidegrees * M_PI / 180000 / 2;

private:
	Servo servo;
	Eigen::Array2d limits;
};

}
