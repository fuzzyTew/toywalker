#pragma once

#include "Vector.hpp"

namespace toywalker {

class Ground
{
public:
	virtual Vector3 projection(Vector3 const & world) = 0;
};

}
