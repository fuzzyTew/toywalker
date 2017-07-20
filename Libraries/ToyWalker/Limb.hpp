#pragma once

#include "ToyWalker.h"


#include "Servo.hpp"
#include "Vector.hpp"

namespace toywalker {

class Body;

class Limb
{
public:
	constexpr static size_t MAX_JOINTS = 8;
	typedef Eigen::Array<Real, Eigen::Dynamic, 1, 0, MAX_JOINTS, 1> Angles;
	typedef Eigen::Array<Servo *, Eigen::Dynamic, 1, 0, MAX_JOINTS, 1> Servos;

	void activate();
	bool activated();
	void deactivate();

	Vector3 const & footGoalBody() const { return _footGoal; }
	Vector3 const & footGoalWorld() const { return _world; }
	Vector3 footBody() { return footBody(angles()); }
	virtual Vector3 footBody(Angles const & plan) = 0;

	Angles const & anglesGoal() const { return _anglesGoal; };
	Angles anglesGoal(Angles const & plan);
	Angles angles();

	bool goalBody(Vector3 const & footDestination);
	virtual Angles planBody(Vector3 const & footDestination) = 0;

	bool goalWorld(Vector3 const & footDestination);
	Angles planWorld(Vector3 const & footDestination);

	Vector3 const & homeBody() const { return _home; }
	Array2 reach() const { return _reach; }

	size_t servos() const { return _servos.size(); }
	Servo & servo(size_t index) { return *_servos[index]; }

	bool attached() { return _contact; }
	void release() { _contact = false; }
	void attach() { _contact = true; }

protected:
	Limb(Body & body, Vector3 const & home, Array2 const & reach, Servos const & servos);
	~Limb();

private:
	const Vector3 _home;
	const Array2 _reach;
	Servos _servos;
	Angles _anglesGoal;
	Vector3 _footGoal;
	Vector3 _world;
	bool _alert;

	Body & _body;
	bool _contact;

	bool handleGoalResult(Angles const & plan);
};

}
