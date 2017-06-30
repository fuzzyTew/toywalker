#pragma once

#include "ServoImplementation.hpp"

#include <Servo.h>

template <int pin, int minMicros, int maxMicros, long long milliDegrees>
class ServoPermanentArduino : public ServoImplementation
{
public:
	ServoPermanentArduino()
	{
		servo.attach(pin, minMicros, maxMicros);
	}

	int id() const
	{
		return pin;
	}

	float goRatio(float ratio)
	{
		int micros = ratio * (maxMicros - minMicros) + 0.5;
		servo.writeMicroseconds(micros + minMicros);
		return micros / float(maxMicros - minMicros);
	}

	float radiansPerRatio() const
	{
		return milliDegrees * M_PI / 180000;
	}

private:
	Servo servo;
};
