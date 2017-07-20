#pragma once

#include "ToyWalker.h"

#include "Geometry.hpp"
#include "Limb.hpp"

namespace toywalker {
	
class Body
{
	friend class Limb;
public:
	constexpr static size_t MAX_LIMBS = 8;

	Body()
	: _worldToBody(Isometry3::Identity()),
	  _bodyToWorld(_worldToBody)
	{ }

	size_t limbs() const { return _limbs.size(); }
	Limb & limb(size_t limb) { return *_limbs[limb]; }

	Isometry3 const & bodyToWorld() const { return _bodyToWorld; }
	Isometry3 const & worldToBody() const { return _worldToBody; }

	void bodyToWorld(Isometry3 const & bodyToWorld)
	{
		_bodyToWorld = bodyToWorld;
		_worldToBody = _bodyToWorld.inverse(Eigen::Isometry);

		for (size_t i = 0; i < limbs(); ++ i)
			if (limb(i).attached()) {
				limb(i).goalWorld(limb(i).footGoalWorld());
			} else {
				limb(i).goalBody(limb(i).footGoalBody());
			}
				
	}

protected:
	virtual void addLimb(Limb * limb)
	{
		_limbs.resize(_limbs.size() + 1);
		_limbs[_limbs.size() - 1] = limb;
	}

	virtual void removeLimb(Limb * limb)
	{
		size_t i;
		for (i = 0; _limbs[i] != limb; ++ i);
		for (; i < _limbs.size() - 1; ++ i) {
			_limbs[i] = _limbs[i + 1];
		}
		_limbs.resize(i);
	}

private:
	Eigen::Array<Limb *, Eigen::Dynamic, 1, 0, MAX_LIMBS, 1> _limbs;
	Isometry3 _worldToBody;
	Isometry3 _bodyToWorld;
};

}
