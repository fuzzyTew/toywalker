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
  _contact(false),
  _step(nullptr)
{
	auto start = millis();
	bool present;
	do {
		present = true;
		for (int i = 0; i < _servos.size(); ++ i) {
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
	_area = _body.bodyToArea() * _footGoal;
	for (int i = 0; i < _servos.size(); ++ i)
		_servos[i]->activate();

	_body.addLimb(this);
}

bool Limb::activated()
{
	for (int i = 0; i < _servos.size(); ++ i)
		if (!_servos[i]->activated())
			return false;
	return true;
}

void Limb::deactivate()
{
	for (int i = 0; i < _servos.size(); ++ i)
		_servos[i]->deactivate();

	_body.removeLimb(this);
}

bool Limb::goalBody(Vector3 const & footDestination)
{
	Angles p = planBody(footDestination);
	if (p.size()) {
		_footGoal = footDestination;
		_area = _body.bodyToArea() * footDestination;
	}
	return handleGoalResult(p);
}

bool Limb::goalArea(Vector3 const & footDestination)
{
	Vector3 footBody = _body.areaToBody() * footDestination;
	Angles p = planBody(footBody);
	if (p.size()) {
		_footGoal = footBody;
		_area = footDestination;
	} else if (attached()) {
		_area = footDestination;
	}
	return handleGoalResult(p);
}

bool Limb::handleGoalResult(Limb::Angles const & p)
{
	if (p.size()) {
		anglesGoal(p);
		if (_alert) {
			_alert = false;
			for (int i = 0; i < _servos.size(); ++ i)
				_servos[i]->alert(false);
		}
		return true;
	} else {
		_alert = true;
		for (int i = 0; i < _servos.size(); ++ i)
			_servos[i]->alert(true);
		return false;
	}
}

Limb::Angles Limb::planArea(Vector3 const & footDestination)
{
	return planBody(_body.areaToBody() * footDestination);
}

Limb::Angles Limb::angles()
{
	Angles ret(_servos.size());
	for (int i = 0; i < _servos.size(); ++ i) {
		_servos[i]->alert(false);
		ret[i] = _servos[i]->angle();
	}
	return ret;
}

Limb::Angles Limb::anglesGoal(Limb::Angles const & plan)
{
	for (int i = 0; i < _servos.size(); ++ i)
		_anglesGoal[i] = _servos[i]->angleGoal(plan[i]);
	return _anglesGoal;
}

}
