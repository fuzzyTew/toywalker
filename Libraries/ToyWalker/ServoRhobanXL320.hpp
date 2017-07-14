#pragma once

#include "ToyWalker.h"

#include "Servo.hpp"

#include <Eigen/Core>

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
	double angle();
	double angleGoal();
	double angleGoal(double radians);
	Eigen::Array2d angleLimit();
	Eigen::Array2d angleLimit(Eigen::Array2d const & radians);
	Eigen::Array2d angleLimitMax();

	bool hasVelocity() { return true; }
	double velocity(); // radians/sec
	double velocityMax() { return VELOCITY_MAX; }
	bool hasVelocityGoal() { return true; }
	double velocityGoal();
	double velocityGoal(double radiansPerSecond);
	bool moving();
	static constexpr double VELOCITY_MAX = M_PI * 114.0 / 30.0;

	bool hasTorque() { return true; }
	double torque(); // newton-decimeters
	double torqueMax() { return TORQUE_MAX; }
	bool hasTorqueLimit() { return true; }
	double torqueLimit();
	double torqueLimit(double newtonDecimeters);
	double torqueLimitMax();
	double torqueLimitMax(double newtonDecimeters);
	static constexpr double TORQUE_MAX = 3.9;

	Eigen::Array3d dynamics(); // position, velocity, torque

	Eigen::Array3d pidGain();
	Eigen::Array3d pidGain(Eigen::Array3d pid);

	// must be called frequently
	static void tick();

	unsigned int modelNumber();
	unsigned int firmwareVersion();

	unsigned long baud();
	static void baudUse(unsigned long baud);
	static void baudSet(unsigned long baud);

	unsigned int temperature();
	unsigned int temperatureLimit();

	double voltage();
	Eigen::Array2d voltageLimit();
	Eigen::Array2d voltageLimit(Eigen::Array2d const & volts);

	bool limitsWithin(bool & voltage, bool & temperature, bool & torque);
	void limitsAlarmed(bool & voltage, bool & temperature, bool & torque);
	void limitsAlarm(bool voltage, bool temperature, bool torque);
	
	struct Color {
		bool red, green, blue;
	};
	Color led();
	void led(Color color);

	double punch();
	double punch(double pct);

	void idBroadcast();
	void idUse(unsigned int id);
	void idSet(unsigned int id);
	unsigned int idAsk();

	static ServoRhobanXL320 & broadcast();

private:
	size_t _id;
};

}
