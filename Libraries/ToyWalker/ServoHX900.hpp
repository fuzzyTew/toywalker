#pragma once

#include "ServoPermanentArduino.hpp"

namespace toywalker {

template <int pin, long long softminMillidegrees = -90000, long long softmaxMillidegrees = 90000>
class ServoPermanentHextronik : public ServoPermanentArduino<pin, 450, 2450, 180000, softminMillidegrees, softmaxMillidegrees>
{
public:
	Real velocityMax() { return VELOCITY_MAX; }
	static constexpr Real VELOCITY_MAX =  60/*deg*/ * M_PI/*rad*/ / 180/*deg*/ / 0.12/*s*/;

	Real torqueMax() { return TORQUE_MAX; }
	static constexpr Real TORQUE_MAX = 1.6/*kg-cm*/ * 9.81/*N/kg*/ / 10/*cm/dm*/;
};

}
