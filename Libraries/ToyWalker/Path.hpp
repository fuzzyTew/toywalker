#pragma once

#include "ToyWalker.h"

#include "Vector.hpp"

#ifdef ARDUINO
	#include <Arduino.h>
#else
	#include <wirish.h>
#endif

namespace toywalker {

template <typename Value>
class Path {
public:
	static constexpr size_t MAX_VALUES = 12;

	virtual Value at(Real seconds) = 0;
	virtual Real duration() { return _duration; }
	virtual void duration(Real seconds) { _duration = seconds; }

	void start() { _start = millis(); }
	Real now() { return (millis() - _start) / Real(1000); }
	void stop() { }

private:
	decltype(millis()) _start;
	Real _duration;
};

}
