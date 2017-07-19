#pragma once

#include "ToyWalker.h"

#include <Eigen/Core>

#include "Servo.hpp"

namespace toywalker {

class Limb
{
friend class Walker;
public:
	constexpr static size_t MAX_JOINTS = 8;
	typedef Eigen::Array<double, Eigen::Dynamic, 1, 0, MAX_JOINTS, 1> Angles;
	typedef Eigen::Array<Servo *, Eigen::Dynamic, 1, 0, MAX_JOINTS, 1> Servos;

	void activate();
	bool activated();
	void deactivate();

	Eigen::Vector3d const & footGoal() const { return _footGoal; }
	Eigen::Vector3d foot() { return foot(angles()); }
	virtual Eigen::Vector3d foot(Angles const & plan) = 0;

	Angles const & anglesGoal() const { return _anglesGoal; };
	Angles anglesGoal(Angles const & plan);
	Angles angles();

	bool goal(Eigen::Vector3d const & footDestination);
	virtual Angles plan(Eigen::Vector3d const & footDestination) = 0;

	Eigen::Vector3d const & home() const { return _home; }
	Eigen::Array2d reach() const { return _reach; }

	size_t servos() const { return _servos.size(); }
	Servo & servo(size_t index) { return *_servos[index]; }

	bool contact() { return _contact; }
	Eigen::Vector3d const & worldContact() { return _worldContact; }

protected:
	Limb(Eigen::Vector3d const & home, Eigen::Array2d reach, Servos const & servos);
	Eigen::Vector3d _footGoal;

private:
	const Eigen::Vector3d _home;
	const Eigen::Array2d _reach;
	Servos _servos;
	Angles _anglesGoal;

	void release() { _contact = false; }
	void contact(Eigen::Vector3d const & world)
	{
		_contact = true;
		_worldContact = world;
	}

	bool _contact;
	Eigen::Vector3d _worldContact;
};

}
