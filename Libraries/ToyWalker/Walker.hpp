#pragma once

#include "Body.hpp"

#include "Ground.hpp"

namespace toywalker {

class Walker : public Body
{
public:
	Walker(Vector3 centerOfMass, Ground & ground)
	: _centerOfMass(centerOfMass),
	  _ground(ground)
	{
		//heightPct(0.5);
	}

	void heightPct(Real pct)
	{
		Array2 range = heightRange();
		height((range[1] - range[0]) * pct + range[0]);
	}

	void height(Real height)
	{
		auto trans = bodyToWorld();
		trans.translation().z() = height;
		bodyToWorld(trans);
	}

	Real height()
	{
		return bodyToWorld().translation().z();
	}

	Array2 heightRange()
	{
		Vector3 bodySky = worldToBody().matrix().topLeftCorner(3,3) * Vector3::UnitZ();
		Array2 ret = {0, INFINITY};
		for (size_t i = 0; i < limbs(); ++ i) {
			auto reach = limb(i).reach();
			Real offset = bodySky.dot(limb(i).homeBody());
			if (reach[0] - offset > ret[0])
				ret[0] = reach[0] - offset;
			if (reach[1] - offset < ret[1])
				ret[1] = reach[1] - offset;
		}
		return ret;
	}

	void position(Vector3 const & position)
	{
		auto trans = bodyToWorld();
		trans.translation() = position;
		bodyToWorld(trans);
	}

	void orientation(Matrix3 orientation)
	{
		auto trans = bodyToWorld();
		trans.matrix().topLeftCorner(3,3) = orientation;
		bodyToWorld(trans);
	}

	Vector3 bodyHome()
	{
		Vector3 ret(0,0,0);
		size_t count = 0;
		for (size_t i = 0; i < limbs(); ++ i)
			if (limb(i).attached()) {
				++ count;
				ret += limb(i).footGoalWorld();
			}
		ret /= count;
		ret.z() = height();
		return ret;
	}

	Ground & ground() { return _ground; }

protected:
	virtual void addLimb(Limb * limb)
	{
		Body::addLimb(limb);

		limb->attach();
		limb->goalWorld(_ground.projection(bodyToWorld() * limb->homeBody()));
	}


private:
	Vector3 _centerOfMass;
	Ground & _ground;
};

}
