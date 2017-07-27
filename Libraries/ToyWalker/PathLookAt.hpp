#pragma once

#include "ToyWalker.h"

#include "Geometry.hpp"
#include "Path.hpp"

namespace toywalker {

class PathLookAt : public Path<Isometry3>
{
public:
	PathLookAt(Path<Vector3> & position, Path<Vector3> & observation, Vector3 const & up);

	virtual Isometry3 at(Real seconds);
	virtual Real duration() { return position().duration(); }
	virtual void duration(Real seconds);

	Path<Vector3> & position() { return *_position; }
	void position(Path<Vector3> & position) { _position = &position; }

	Path<Vector3> & observation() { return *_observation; }
	void observation(Path<Vector3> & observation) { _observation = &observation; }

	Vector3 const & up() { return _up; }
	void up(Vector3 const & up) { _up = up; }

private:
	Path<Vector3> * _position;
	Path<Vector3> * _observation;
	Vector3 _up;
};

}

// impl

namespace toywalker {

PathLookAt::PathLookAt(Path<Vector3> & position, Path<Vector3> & observation, Vector3 const & up)
: _position(&position), _observation(&observation), _up(up)
{ }

Isometry3 PathLookAt::at(Real seconds)
{
	Vector3 pos = position().at(seconds);
	Vector3 obs = observation().at(seconds);

	Isometry3 ret;

	ret.affine().col(0) = (obs - pos).normalized();
	ret.affine().col(1) = up().cross(ret.affine().col(0));
	ret.affine().col(2) = ret.affine().col(0).cross(ret.affine().col(1));
	ret.affine().col(3) = pos;

	return ret;
}

void PathLookAt::duration(Real seconds)
{
	position().duration(seconds);
	observation().duration(seconds);
}

}
