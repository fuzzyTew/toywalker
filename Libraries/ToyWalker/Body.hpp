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
	: _areaToBody(Isometry3::Identity()),
	  _bodyToArea(_areaToBody)
	{ }

	size_t limbs() const { return _limbs.size(); }
	Limb & limb(size_t limb) { return *_limbs[limb]; }

	Isometry3 const & bodyToArea() const { return _bodyToArea; }
	Isometry3 const & areaToBody() const { return _areaToBody; }

	void bodyToArea(Isometry3 const & bodyToArea)
	{
		_bodyToArea = bodyToArea;
		_areaToBody = _bodyToArea.inverse(Eigen::Isometry);

		for (size_t i = 0; i < limbs(); ++ i)
			if (limb(i).attached()) {
				limb(i).goalArea(limb(i).footGoalArea());
			// TODO: I've commented this out because it disallows us
			//       from moving detached limbs.  But now detached limbs
			//       do not have an up-to-date area location if they are
			//       not moved.  Perhaps update it.
			//} else {
			//	limb(i).goalBody(limb(i).footGoalBody());
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
		int i;
		for (i = 0; _limbs[i] != limb; ++ i);
		for (; i < _limbs.size() - 1; ++ i) {
			_limbs[i] = _limbs[i + 1];
		}
		_limbs.resize(i);
	}

private:
	ArrayX<MAX_LIMBS, Limb *> _limbs;
	Isometry3 _areaToBody;
	Isometry3 _bodyToArea;
};

}
