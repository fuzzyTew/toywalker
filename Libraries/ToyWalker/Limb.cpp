#include "Limb.hpp"

#include "Body.hpp"
#include "Vector.hpp"

#include <wirish.h>
#include <terminal.h>

namespace toywalker {

Limb::Limb(Body & body, Vector3 const & home, Array2 const & reach, Servos const & servos)
: _home(home),
  _reach(reach),
  _servos(servos),
  _anglesGoal(_servos.size()),
  _alert(false),
  _body(body),
  _contact(false)
{
	auto start = millis();
	bool present;
	do {
		present = true;
		for (size_t i = 0; i < _servos.size(); ++ i) {
			terminal_tick();
			if (!_servos[i]->present()) {
				present = false;
				break;
			}
		}
	} while(!present && start + 200 < millis());
}

Limb::~Limb()
{
	_body.removeLimb(this);
}

void Limb::activate()
{
	anglesGoal(angles());
	_footGoal = footBody(_anglesGoal);
	_world = _body.bodyToWorld() * _footGoal;
	for (size_t i = 0; i < _servos.size(); ++ i)
		_servos[i]->activate();

	_body.addLimb(this);
}

bool Limb::activated()
{
	for (size_t i = 0; i < _servos.size(); ++ i)
		if (!_servos[i]->activated())
			return false;
	return true;
}

void Limb::deactivate()
{
	for (size_t i = 0; i < _servos.size(); ++ i)
		_servos[i]->deactivate();

	_body.removeLimb(this);
}

bool Limb::goalBody(Vector3 const & footDestination)
{
	Angles p = planBody(footDestination);
	if (p.size()) {
		_footGoal = footDestination;
		_world = _body.bodyToWorld() * footDestination;
	}
	return handleGoalResult(p);
}

bool Limb::goalWorld(Vector3 const & footDestination)
{
	Vector3 footBody = _body.worldToBody() * footDestination;
	Angles p = planBody(footBody);
	if (p.size()) {
		_footGoal = footBody;
		_world = footDestination;
	} else if (attached()) {
		_world = footDestination;
	}
	return handleGoalResult(p);
}

bool Limb::handleGoalResult(Limb::Angles const & p)
{
	if (p.size()) {
		anglesGoal(p);
		if (_alert) {
			_alert = false;
			for (size_t i = 0; i < _servos.size(); ++ i)
				_servos[i]->alert(false);
		}
		return true;
	} else {
		_alert = true;
		for (size_t i = 0; i < _servos.size(); ++ i)
			_servos[i]->alert(true);
		return false;
	}
}

Limb::Angles Limb::planWorld(Vector3 const & footDestination)
{
	return planBody(_body.worldToBody() * footDestination);
}

Limb::Angles Limb::angles()
{
	Angles ret(_servos.size());
	for (size_t i = 0; i < _servos.size(); ++ i) {
		_servos[i]->alert(false);
		ret[i] = _servos[i]->angle();
	}
	return ret;
}

Limb::Angles Limb::anglesGoal(Limb::Angles const & plan)
{
	for (size_t i = 0; i < _servos.size(); ++ i)
		_anglesGoal[i] = _servos[i]->angleGoal(plan[i]);
	return _anglesGoal;
}

}
