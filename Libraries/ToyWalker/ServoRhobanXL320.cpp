#include "ServoRhobanXL320.hpp"

#ifdef HAS_DXL

#include <dxl.h>
#include <wirish/wirish.h>

namespace toywalker {

static ServoRhobanXL320 broadcast(DXL_BROADCAST);

enum Baud {
	BAUD9600 = 0,
	BAUD57600 = 1,
	BAUD115200 = 2,
	BAUD1MBPS = 3
};

Baud baudToEnum(unsigned long baud)
{
	if (baud < (9600+57600)/2)
		return BAUD9600;
	if (baud < (57600+115200)/2)
		return BAUD57600;
	if (baud < (115200+1000000)/2)
		return BAUD115200;
	return BAUD1MBPS;
}

unsigned long enumToBaud(Baud baud)
{
	switch (baud) {
	case BAUD9600: return 9600;
	case BAUD57600: return 57600;
	case BAUD115200: return 115200;
	case BAUD1MBPS: return 1000000;
	default: return 0;
	}
}

#pragma pack(1)
struct XL320
{
	struct EEPROM {
		uint16_t modelNumber;
		uint8_t firmwareVersion;
		uint8_t id;
		uint8_t baudRate;
		uint8_t returnDelayTime;
		uint16_t angleLimits[2];
		uint8_t __internal;
		uint8_t controlMode;
		uint8_t limitTemperature;
		uint8_t voltageLimits[2];
		uint16_t maxTorque;
		uint8_t returnLevel;
		uint8_t alarmShutdown;
		uint8_t __internal2[5];
	} eeprom;

	// RAM
	struct RAM {
		uint8_t torqueEnable;
		uint8_t led;
		uint8_t __internal;
		struct Gain {
			uint8_t derivative;
			uint8_t integral;
			uint8_t proportional;
		} gain;
		struct Goal {
			uint16_t position;
			uint16_t velocity;
			uint8_t __internal;
			uint16_t torque;
		} goal;
		struct State {
			struct Dynamics {
				uint16_t position;
				uint16_t speed;
				uint16_t load;
			} dynamics;
			uint16_t __internal;
			uint8_t voltage;
			uint8_t temperature;
			uint8_t registeredInstruction;
			uint8_t __internal2;
			uint8_t moving;
			uint8_t hardwareErrorStatus;
		} state;
		uint16_t punch;
	} ram;
} xl320;

#define WRITE(what) \
	dxl_write(_id, (uint8_t*)&what - (uint8_t*)&xl320, (char*)&what, sizeof(what)); \
	delay(5);

#define READ(what) \
	((dxl_read(_id, (uint8_t*)&what - (uint8_t*)&xl320, (char*)&what, sizeof(what)), what))

void ServoRhobanXL320::baudUse(unsigned long baud)
{
	dxl_init(enumToBaud(baudToEnum(baud)));
	dxl_configure_all();
}

void ServoRhobanXL320::baudSet(unsigned long baud)
{
	xl320.eeprom.baudRate = baudToEnum(baud);
	unsigned int _id = broadcast().id();
	WRITE(xl320.eeprom.baudRate);
	dxl_init(enumToBaud(Baud(xl320.eeprom.baudRate)));
}

void ServoRhobanXL320::tick()
{
	dxl_tick();
}

bool ServoRhobanXL320::present()
{
	return dxl_ping(_id);
}

ServoRhobanXL320::ServoRhobanXL320()
: _id(DXL_BROADCAST)
{ }

ServoRhobanXL320::ServoRhobanXL320(unsigned int id)
: _id(id)
{ }

void ServoRhobanXL320::idBroadcast()
{
	idUse(DXL_BROADCAST);
}

void ServoRhobanXL320::idUse(unsigned int id)
{
	_id = id;
}

void ServoRhobanXL320::idSet(unsigned int id)
{
	dxl_configure(_id, id);
	_id = id;
}

unsigned int ServoRhobanXL320::idAsk()
{
	return READ(xl320.eeprom.id);
}

ServoRhobanXL320 & ServoRhobanXL320::broadcast()
{
	return toywalker::broadcast;
}

void ServoRhobanXL320::activate()
{
	xl320.ram.torqueEnable = 1;
	WRITE(xl320.ram.torqueEnable);
	xl320.ram.goal.torque = 1023;
	WRITE(xl320.ram.goal.torque);
}

void ServoRhobanXL320::deactivate()
{
	xl320.ram.torqueEnable = 0;
	WRITE(xl320.ram.torqueEnable);
}

constexpr double angles_per_radian = 1023.0 * 180.0 / 300.0 / M_PI;
constexpr double radians_per_angle = M_PI * 300.0 / 180.0 / 1023.0;
constexpr uint16_t angle_center = 512;

constexpr double radianspersecond_per_speed = ServoRhobanXL320::VELOCITY_MAX / 1023.0;
constexpr double speed_per_radianspersecond = 1023.0 / ServoRhobanXL320::VELOCITY_MAX;

constexpr double newtondecimeters_per_load = 3.9 / 1023.0;
constexpr double loads_per_newtondecimeter = 1023.0 / 3.9;

double ServoRhobanXL320::angleGoal(double radians)
{
	xl320.ram.goal.position = radians * angles_per_radian + 0.5 + angle_center;
	WRITE(xl320.ram.goal.position);
	return (xl320.ram.goal.position - angle_center) * radians_per_angle;
}

double ServoRhobanXL320::angle()
{
	return (READ(xl320.ram.state.dynamics.position) - angle_center) * radians_per_angle;
}

double ServoRhobanXL320::velocity()
{
	double speed = (READ(xl320.ram.state.dynamics.speed) & 1023) * radianspersecond_per_speed;
	if (xl320.ram.state.dynamics.speed & 1024)
		return -speed;
	else
		return speed;
}

double ServoRhobanXL320::torque()
{
	double torque = (READ(xl320.ram.state.dynamics.load) & 1023) * newtondecimeters_per_load;
	if (xl320.ram.state.dynamics.load & 1024)
		return -torque;
	else
		return torque;
}

Eigen::Array3d ServoRhobanXL320::dynamics()
{
	READ(xl320.ram.state.dynamics);
	return {
		(xl320.ram.state.dynamics.position - angle_center) * radians_per_angle,
		(xl320.ram.state.dynamics.speed & 1024) ?
			-(xl320.ram.state.dynamics.speed & 1023) * radianspersecond_per_speed :
			xl320.ram.state.dynamics.speed * radianspersecond_per_speed,
		(xl320.ram.state.dynamics.load & 1024) ?
			-(xl320.ram.state.dynamics.load & 1023) * newtondecimeters_per_load :
			xl320.ram.state.dynamics.load * newtondecimeters_per_load
	};
}

unsigned int ServoRhobanXL320::modelNumber()
{
	return READ(xl320.eeprom.modelNumber);
}

unsigned int ServoRhobanXL320::firmwareVersion()
{
	return READ(xl320.eeprom.firmwareVersion);
}

unsigned long ServoRhobanXL320::baud()
{
	return enumToBaud(Baud(READ(xl320.eeprom.baudRate)));
}

Eigen::Array2d ServoRhobanXL320::angleLimit()
{
	READ(xl320.eeprom.angleLimits);
	return {
		(xl320.eeprom.angleLimits[0] - angle_center) * radians_per_angle,
		(xl320.eeprom.angleLimits[1] - angle_center) * radians_per_angle
	};
}

Eigen::Array2d ServoRhobanXL320::angleLimit(Eigen::Array2d const & radians)
{
	xl320.eeprom.angleLimits[0] = radians[0] * angles_per_radian + 0.5 + angle_center;
	xl320.eeprom.angleLimits[1] = radians[1] * angles_per_radian + 0.5 + angle_center;
	WRITE(xl320.eeprom.angleLimits);
	return {
		(xl320.eeprom.angleLimits[0] - angle_center) * radians_per_angle,
		(xl320.eeprom.angleLimits[1] - angle_center) * radians_per_angle
	};
}

Eigen::Array2d ServoRhobanXL320::angleLimitMax()
{
	return {
		(0 - angle_center) * radians_per_angle,
		(1023 - angle_center) * radians_per_angle
	};
}

unsigned int ServoRhobanXL320::temperatureLimit()
{
	return READ(xl320.eeprom.limitTemperature);
}

Eigen::Array2d ServoRhobanXL320::voltageLimit()
{
	READ(xl320.eeprom.voltageLimits);
	return {
		xl320.eeprom.voltageLimits[0] * 0.1,
		xl320.eeprom.voltageLimits[1] * 0.1
	};
}

Eigen::Array2d ServoRhobanXL320::voltageLimit(Eigen::Array2d const & volts)
{
	xl320.eeprom.voltageLimits[0] = volts[0] * 10 + 0.5;
	xl320.eeprom.voltageLimits[1] = volts[1] * 10 + 0.5;
	WRITE(xl320.eeprom.voltageLimits);
	return {
		xl320.eeprom.voltageLimits[0] * 0.1,
		xl320.eeprom.voltageLimits[1] * 0.1
	};
}

double ServoRhobanXL320::torqueLimitMax()
{
	READ(xl320.eeprom.maxTorque);
	return xl320.eeprom.maxTorque * newtondecimeters_per_load;
}

double ServoRhobanXL320::torqueLimitMax(double newtonDecimeters)
{
	xl320.eeprom.maxTorque = newtonDecimeters * loads_per_newtondecimeter + 0.5;
	WRITE(xl320.eeprom.maxTorque);
	return xl320.eeprom.maxTorque * newtondecimeters_per_load;
}

void ServoRhobanXL320::limitsAlarmed(bool & voltage, bool & temperature, bool & torque)
{
	READ(xl320.eeprom.alarmShutdown);
	voltage = xl320.eeprom.alarmShutdown & 0x04;
	temperature = xl320.eeprom.alarmShutdown & 0x02;
	torque = xl320.eeprom.alarmShutdown & 0x01;
}

void ServoRhobanXL320::limitsAlarm(bool voltage, bool temperature, bool torque)
{
	xl320.eeprom.alarmShutdown =
		(voltage ? 0x04 : 0x00) |
		(temperature ? 0x02: 0x00) |
		(torque ? 0x01 : 0x00);
	WRITE(xl320.eeprom.alarmShutdown);
}

bool ServoRhobanXL320::activated()
{
	return READ(xl320.ram.torqueEnable) && READ(xl320.ram.goal.torque) != 0;
}

ServoRhobanXL320::Color ServoRhobanXL320::led()
{
	READ(xl320.ram.led);
	return {xl320.ram.led & 0x01, xl320.ram.led & 0x02, xl320.ram.led & 0x04};
}

void ServoRhobanXL320::led(ServoRhobanXL320::Color color)
{
	xl320.ram.led =
		(color.red ? 0x01 : 0x00) |
		(color.green ? 0x02 : 0x00) |
		(color.blue ? 0x04 : 0x00);
	WRITE(xl320.ram.led);
}

Eigen::Array3d ServoRhobanXL320::pidGain()
{
	READ(xl320.ram.gain);
	return {
		xl320.ram.gain.proportional / 8.0,
		xl320.ram.gain.integral * 1000.0 / 2048.0,
		xl320.ram.gain.derivative * 4.0 / 1000.0
	};
}

Eigen::Array3d ServoRhobanXL320::pidGain(Eigen::Array3d pid)
{
	xl320.ram.gain.proportional = pid[0] * 8.0 + 0.5;
	xl320.ram.gain.integral = pid[1] * 2048.0 / 1000.0;
	xl320.ram.gain.derivative = pid[2] * 1000.0 / 4.0;
	WRITE(xl320.ram.gain);
	return {
		xl320.ram.gain.proportional / 8.0,
		xl320.ram.gain.integral * 1000.0 / 2048.0,
		xl320.ram.gain.derivative * 4.0 / 1000.0
	};
}

double ServoRhobanXL320::angleGoal()
{
	return (READ(xl320.ram.goal.position) - angle_center) * radians_per_angle;
}

double ServoRhobanXL320::velocityGoal()
{
	return READ(xl320.ram.goal.velocity) * radianspersecond_per_speed;
}

double ServoRhobanXL320::velocityGoal(double radiansPerSecond)
{
	xl320.ram.goal.velocity = radiansPerSecond * speed_per_radianspersecond + 0.5;
	WRITE(xl320.ram.goal.velocity);
	return xl320.ram.goal.velocity * radianspersecond_per_speed;
}

double ServoRhobanXL320::torqueLimit()
{
	return READ(xl320.ram.goal.torque) * newtondecimeters_per_load;
}

double ServoRhobanXL320::torqueLimit(double newtonDecimeters)
{
	xl320.ram.goal.torque = newtonDecimeters * loads_per_newtondecimeter + 0.5;
	WRITE(xl320.ram.goal.torque);
	return xl320.ram.goal.torque * newtondecimeters_per_load;
}

double ServoRhobanXL320::voltage()
{
	return READ(xl320.ram.state.voltage) * 0.1;
}

unsigned int ServoRhobanXL320::temperature()
{
	return READ(xl320.ram.state.temperature);
}

bool ServoRhobanXL320::moving()
{
	return READ(xl320.ram.state.moving);
}

bool ServoRhobanXL320::limitsWithin(bool & voltage, bool & temperature, bool & torque)
{
	READ(xl320.ram.state.hardwareErrorStatus);
	voltage = !(xl320.ram.state.hardwareErrorStatus & 0x04);
	temperature = !(xl320.ram.state.hardwareErrorStatus & 0x02);
	torque = !(xl320.ram.state.hardwareErrorStatus & 0x01);
	return !xl320.ram.state.hardwareErrorStatus;
}

double ServoRhobanXL320::punch()
{
	return (READ(xl320.ram.punch) - 32.0) / (1023.0 - 32.0);
}

double ServoRhobanXL320::punch(double pct)
{
	xl320.ram.punch = pct * (1023.0 - 32.0) + 32.0 + 0.5;
	WRITE(xl320.ram.punch);
	return (xl320.ram.punch - 32.0) / (1023.0 - 32.0);
}

}

#endif

