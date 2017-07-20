#pragma once

#include "Ground.hpp"

#include "Geometry.hpp"
#include "Vector.hpp"

namespace toywalker {

class GroundPlane : public Ground
{
public:
	GroundPlane(Plane plane = {Vector3::UnitZ(), 0})
	: _plane(plane)
	{ }

	Vector3 projection(Vector3 const & world)
	{
		return _plane.projection(world);
	}

private:
	Plane _plane;
};

}
