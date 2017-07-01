#pragma once

#include "ServoPermanentArduino.hpp"

template <int pin, long long softminMillidegrees = -90000, long long softmaxMillidegrees = 90000>
class ServoPermanentHextronik : public ServoPermanentArduino<pin, 450, 2450, 180000, softminMillidegrees, softmaxMillidegrees>
{
};
