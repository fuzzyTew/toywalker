#pragma once

#include "ToyWalker.h"

#include <Eigen/Core>

class ServoRhobanXL320
{
public:
	ServoRhobanXL320();
	ServoRhobanXL320(unsigned int id);
	void activate();
	void deactivate();
	unsigned int id() const { return _id; }
	double go(double radians);
	double where();
	double softmin() const { return -150.0 * 180.0 / M_PI; }
	double softmax() const { return 150.0 * 180.0 / M_PI; }

	double speed(); // radians/sec
	double torque(); // newton-decimeters
	Eigen::Array3d dynamics(); // position, speed, torque

	// must be called frequently
	static void tick();

	bool ping();

	unsigned int modelNumber();
	unsigned int firmwareVersion();
	unsigned long baud();
	Eigen::Array2d angleLimit();
	Eigen::Array2d angleLimit(Eigen::Array2d const & radians);
	unsigned int temperatureLimit();
	Eigen::Array2d voltageLimit();
	Eigen::Array2d voltageLimit(Eigen::Array2d const & volts);
	double maxTorqueLimit();
	double maxTorqueLimit(double newtonDecimeters);
	void outOfRangeDetected(bool & voltage, bool & temperature, bool & torque);
	void detectOutOfRange(bool voltage, bool temperature, bool torque);
	
	bool activated();
	struct Color {
		bool red, green, blue;
	};
	Color led();
	void led(Color color);
	Eigen::Array3d pidGain();
	Eigen::Array3d pidGain(Eigen::Array3d pid);
	double whereGoal();
	double goalSpeed();
	double goalSpeed(double radiansPerSecond);
	double torqueLimit();
	double torqueLimit(double newtonDecimeters);
	double voltage();
	unsigned int temperature();
	bool moving();
	bool stateInRange(bool & voltage, bool & temperature, bool & torque);
	double punch();
	double punch(double pct);

	static void useBaud(unsigned long baud);
	static void setBaud(unsigned long baud);

	void useBroadcast();
	void useId(unsigned int id);
	void setId(unsigned int id);
	unsigned int askId();

	static ServoRhobanXL320 & broadcast();

private:
	unsigned int _id;
};
