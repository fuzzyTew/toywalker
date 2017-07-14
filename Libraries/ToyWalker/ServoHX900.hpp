#pragma once

#include "ServoPermanentArduino.hpp"

namespace toywalker {

template <int pin, long long softminMillidegrees = -90000, long long softmaxMillidegrees = 90000>
class ServoPermanentHextronik : public ServoPermanentArduino<pin, 450, 2450, 180000, softminMillidegrees, softmaxMillidegrees>
{
public:
	double velocityMax() { return VELOCITY_MAX; }
	static constexpr double VELOCITY_MAX =  60/*deg*/ * M_PI/*rad*/ / 180/*deg*/ / 0.12/*s*/;

	double torqueMax() { return TORQUE_MAX; }
	static constexpr double TORQUE_MAX = 1.6/*kg-cm*/ * 9.81/*N/kg*/ / 10/*cm/dm*/;
};

}
