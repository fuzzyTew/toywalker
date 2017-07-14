#pragma once

#include <initializer_list>
#include <tuple>

#include <Eigen/Geometry>

#include "Limb.hpp"

namespace toywalker {

class Walker
{
public:
	typedef Eigen::Array<Limb *, Eigen::Dynamic, 1, 0, 8, 1> Limbs;

	template <typename... _Limb>
	Walker(Eigen::Array2d spreadRange,
	       Eigen::Array2d heightRange,
	       Eigen::Vector3d centerOfMass,
	       _Limb &... limbs)
	: bodyFrame(Eigen::Matrix4d::Identity()),
	  spreadRange(spreadRange),
	  heightRange(heightRange),
	  centerOfMass(centerOfMass),
	  _limbs(Limbs::Map(std::initializer_list<Limb *>({&limbs...}).begin(),sizeof...(limbs)))
	{ }

	unsigned int limbs()
	{
		return _limbs.size();
	}

	Limb & limb(unsigned int limb)
	{
		return *_limbs[limb];
	}

	unsigned int servos()
	{
		unsigned int ret = 0;
		for (int i = 0; i < _limbs.size(); ++ i)
			ret += _limbs(i)->servos();
		return ret;
	}

	void servoGo(unsigned int servo, double radians)
	{
		int curServo = 0;
		for (int curLimb = 0;
		     curLimb < _limbs.size();
		     curServo += _limbs[curLimb ++]->servos()
		    )
			if (curServo + _limbs[curLimb]->servos() > servo) {
				_limbs[curLimb]->go(servo - curServo, radians);
				return;
			}
	}

private:
	Eigen::Matrix4d bodyFrame;
	Eigen::Array2d spreadRange;
	Eigen::Array2d heightRange;
	Eigen::Vector3d centerOfMass;
	Limbs _limbs;
};

}
