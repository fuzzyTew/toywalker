#pragma once

#include "ServoPermanentArduino.hpp"

template <int pin>
class ServoPermanentHextronik : public ServoPermanentArduino<pin, 450, 2450, 180000>
{
};
