#pragma once

#include "ToyWalker.h"

#include "Servo.hpp"
#include "Vector.hpp"

namespace toywalker {

class ServoRhobanXL320 : public Servo
{
public:
	ServoRhobanXL320();
	ServoRhobanXL320(size_t id);

	void activate();
	void deactivate();
	bool activated();
	bool present();

	size_t id() { return _id; }

	bool hasAngle() { return true; }
	Real angle();
	Real angleGoal();
	Real angleGoal(Real radians);
	Array2 angleLimit();
	Array2 angleLimit(Array2 const & radians);
	Array2 angleLimitMax();

	bool hasVelocity() { return true; }
	Real velocity(); // radians/sec
	Real velocityMax() { return VELOCITY_MAX; }
	bool hasVelocityGoal() { return true; }
	Real velocityGoal();
	Real velocityGoal(Real radiansPerSecond);
	bool moving();
	static constexpr Real VELOCITY_MAX = M_PI * 114.0 / 30.0;

	bool hasTorque() { return true; }
	Real torque(); // newton-decimeters
	Real torqueMax() { return TORQUE_MAX; }
	bool hasTorqueLimit() { return true; }
	Real torqueLimit();
	Real torqueLimit(Real newtonDecimeters);
	Real torqueLimitMax();
	Real torqueLimitMax(Real newtonDecimeters);
	static constexpr Real TORQUE_MAX = 3.9;

	Array3 dynamics(); // position, velocity, torque

	bool hasAlert() { return true; }
	void alert(bool alert);

	Array3 pidGain();
	Array3 pidGain(Array3 pid);

	// must be called frequently
	static void tick();

	unsigned int modelNumber();
	unsigned int firmwareVersion();

	unsigned long baud();
	static void baudUse(unsigned long baud);
	static void baudSet(unsigned long baud);

	unsigned int temperature();
	unsigned int temperatureLimit();

	Real voltage();
	Array2 voltageLimit();
	Array2 voltageLimit(Array2 const & volts);

	bool limitsWithin(bool & voltage, bool & temperature, bool & torque);
	void limitsAlarmed(bool & voltage, bool & temperature, bool & torque);
	void limitsAlarm(bool voltage, bool temperature, bool torque);
	
	struct Color {
		bool red, green, blue;
	};
	Color led();
	void led(Color color);

	Real punch();
	Real punch(Real pct);

	void idBroadcast();
	void idUse(unsigned int id);
	void idSet(unsigned int id);
	unsigned int idAsk();

	static ServoRhobanXL320 & broadcast();

private:
	size_t _id;
};

}
