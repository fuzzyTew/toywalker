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
	typedef Eigen::Hyperplane<double, 3> Plane;
	typedef Eigen::Affine3d Transform;

	template <typename... _Limb>
	Walker(Eigen::Vector3d centerOfMass,
	       _Limb &... limbs)
	: _limbs(Limbs::Map(std::initializer_list<Limb *>({&limbs...}).begin(),sizeof...(limbs))),
	  _centerOfMass(centerOfMass),
	  _worldGround({0,0,1}, 0),
	  _bodyGround(_worldGround),
	  _worldToBody(Transform::Identity()),
	  _bodyToWorld(Transform::Identity())
	{
		heightPct(0.5);

		for (size_t i = 0; i < this->limbs(); ++ i) {
			limb(i).goal(_bodyGround.projection(limb(i).home()));
			limb(i).contact(_bodyToWorld * limb(i).footGoal());
		}
	}

	size_t limbs()
	{
		return _limbs.size();
	}

	Limb & limb(size_t limb)
	{
		return *_limbs[limb];
	}

	size_t servos()
	{
		size_t ret = 0;
		for (size_t i = 0; i < _limbs.size(); ++ i)
			ret += _limbs(i)->servos();
		return ret;
	}

	Servo & servo(size_t servo)
	{
		size_t curServo = 0;
		for (size_t curLimb = 0;
		     curLimb < _limbs.size();
		     curServo += _limbs[curLimb ++]->servos()
		    )
			if (curServo + _limbs[curLimb]->servos() > servo)
				return _limbs[curLimb]->servo(servo - curServo);
	}

	void heightPct(double pct)
	{
		Eigen::Array2d range = heightRange();
		height((range[1] - range[0]) * pct + range[0]);
	}

	void height(double height)
	{
		Eigen::Vector3d worldSky = _worldGround.normal();
		_bodyToWorld.translation() += worldSky * (height - worldSky.dot(_bodyToWorld.translation()));

		updateCache();
	}

	Eigen::Array2d heightRange()
	{
		Eigen::Vector3d bodyGravity = -_bodyGround.normal();
		Eigen::Array2d ret = {0, INFINITY};
		for (size_t i = 0; i < limbs(); ++ i) {
			auto reach = limb(i).reach();
			double offset = bodyGravity.dot(limb(i).home());
			if (offset + reach[0] > ret[0])
				ret[0] = offset + reach[0];
			if (offset + reach[1] < ret[1])
				ret[1] = offset + reach[1];
		}
		return ret;
	}

private:
	Limbs _limbs;
	Eigen::Vector3d _centerOfMass;
	Plane _worldGround, _bodyGround;
	Transform _worldToBody, _bodyToWorld;

	void updateCache()
	{
		_worldToBody = _bodyToWorld.inverse();
		_bodyGround = _worldGround;
		_bodyGround.transform(_worldToBody);

		updateContacts();
	}

	void updateContacts()
	{
		for (size_t i = 0; i < limbs(); ++ i)
			if (limb(i).contact())
				limb(i).goal(_worldToBody * limb(i).worldContact());
	}
};

}
