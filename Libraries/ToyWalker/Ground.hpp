#pragma once

#include "ToyWalker.h"

#include "Vector.hpp"

namespace toywalker {

class Ground
{
public:
	virtual Vector3 projection(Vector3 const & area) = 0;
};

}
