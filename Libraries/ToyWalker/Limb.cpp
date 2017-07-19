#include "Limb.hpp"

#include <wirish.h>
#include <terminal.h>

namespace toywalker {

Limb::Limb(Eigen::Vector3d const & home, Eigen::Array2d reach, Servos const & servos)
: _home(home),
  _reach(reach),
  _servos(servos),
  _anglesGoal(_servos.size()),
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

void Limb::activate()
{
	anglesGoal(angles());
	_footGoal = foot(_anglesGoal);
	for (size_t i = 0; i < _servos.size(); ++ i)
		_servos[i]->activate();
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
}

bool Limb::goal(Eigen::Vector3d const & footDestination)
{
	Angles p = plan(footDestination);
	if (p.size()) {
		_footGoal = footDestination;
		anglesGoal(p);
		return true;
	} else {
		for (size_t i = 0; i < _servos.size(); ++ i)
			_servos[i]->alert(true);
		return false;
	}
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
