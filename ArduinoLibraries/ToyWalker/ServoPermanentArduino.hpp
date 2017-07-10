#pragma once

#include <Servo.h>

template <int pin, int minMicros, int maxMicros, long long extentMillidegrees, long long softminMillidegrees, long long softmaxMillidegrees>
class ServoPermanentArduino
{
public:
	ServoPermanentArduino()
	{
		activate();
		go(0);
	}

	void activate()
	{
		servo.attach(pin, minMicros, maxMicros);
	}

	int id() const
	{
		return pin;
	}

	double go(double radians)
	{
		int micros = radians * (180000.0 * (maxMicros - minMicros) / extentMillidegrees / M_PI);
		servo.writeMicroseconds(micros + (maxMicros + minMicros) / 2);
		return _radians = micros * (M_PI * extentMillidegrees / (maxMicros - minMicros) / 180000);
	}

	double where() const
	{
		return _radians;
	}

	double softmin() const
	{
		return softminMillidegrees * M_PI / 180000;
	}

	double softmax() const
	{
		return softmaxMillidegrees * M_PI / 180000;
	}

private:
	Servo servo;
	double _radians;
};
