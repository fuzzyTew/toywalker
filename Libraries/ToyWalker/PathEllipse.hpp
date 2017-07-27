#pragma once

#include "ToyWalker.h"

#include "Geometry.hpp"
#include "Path.hpp"
#include "Vector.hpp"

namespace toywalker {

class PathEllipse : public Path<Vector3>
{
public:
	PathEllipse(Vector3 axis1, Vector3 axis2, Vector3 center, Real duration);

	Vector3 at(Real seconds);

	Vector3 axis1() { return transform.affine().col(0); }
	void axis1(Vector3 const & axis1) { transform.affine().col(0) = axis1; }

	Vector3 axis2() { return transform.affine().col(1); }
	void axis2(Vector3 const & axis2) { transform.affine().col(1) = axis2; }

	Vector3 center() { return transform.affine().col(3); }
	void center(Vector3 const & center) { transform.affine().col(3) = center; }

private:
	Affine3 transform;
};

}

// impl

namespace toywalker {

PathEllipse::PathEllipse(Vector3 axis1, Vector3 axis2, Vector3 center, Real duration)
{
	// we will plot a unit circle and transform it by the passed parameters

	transform.affine() << axis1, axis2, axis1.cross(axis2), center;

	this->duration(duration);
}

Vector3 PathEllipse::at(Real seconds)
{
	Real angle = seconds * 2 * M_PI / duration();
	return transform * Vector3(cos(angle), sin(angle), 0);
}

}
