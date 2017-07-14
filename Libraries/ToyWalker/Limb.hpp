#pragma once

#include <Eigen/Core>

namespace toywalker {

class Limb
{
public:
	constexpr size_t MAX_JOINTS = 8;
	typedef Eigen::Array<double, Eigen::Dynamic, 1, 0, MAX_JOINTS, 1> Angles;

	virtual void go(Eigen::Vector3d const &) = 0;

	virtual Angles plan(Eigen::Vector3d const &) = 0;
	virtual void execute(Angles const & plan) = 0;

	virtual Eigen::Vector3d foot() = 0;
	virtual Eigen::Vector3d foot(Angles const & plan) = 0;

	Eigen::Vector3d hip() { return _hip; }

	virtual unsigned int servos() { return _servos; }

	virtual double go(unsigned int servo, double radians) = 0;

	virtual Angles servoRadians() = 0;
	virtual double servoRadians(unsigned int servo) = 0;

protected:
	Limb(Eigen::Vector3d const & hip, unsigned int const & servos)
	: _hip(hip), _servos(servos)
	{ }

	const Eigen::Vector3d _hip;
	const unsigned int _servos;
};

}
