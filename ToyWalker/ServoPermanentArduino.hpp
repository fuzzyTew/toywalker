#pragma once

#include <Servo.h>

template <int pin, int minMicros, int maxMicros, long long milliDegrees>
class ServoPermanentArduino
{
public:
	ServoPermanentArduino()
	{
		servo.attach(pin, minMicros, maxMicros);
	}

	float goRatio(float ratio)
	{
		int micros = ratio * (maxMicros - minMicros) + 0.5;
		servo.writeMicroseconds(micros + minMicros);
		return micros / float(maxMicros - minMicros);
	}

private:
	Servo servo;
};
