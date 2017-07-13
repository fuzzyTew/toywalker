#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <main.h>

// TODO: remove
#include <dxl.h>

#include <ToyWalker.h>
#include <ServoRhobanXL320.hpp>

unsigned numServos = 0;

TERMINAL_PARAMETER_INT(id, "Servo ID", 0);

ServoRhobanXL320 servo(id);


TERMINAL_COMMAND(setid, "Sets id of connected servo")
{
	numServos = 0;
	for (unsigned int i = 0; i < 253; ++ i) {
	        servo.useId(i);
	        if (servo.ping()) {
	    	    ++ numServos;
	        }
	}
	if (numServos != 1) {
		terminal_io()->println("ERR: Require exactly 1 connected servo.");
		return;
	}
	unsigned int id;
	if (argc > 1) {
		terminal_io()->println("Usage: setid id");
		return;
	} else if (argc == 1) {
		id = atoi(argv[0]);
	} else {
		id = ::id;
	}
	servo.useBroadcast();
	servo.setId(id);
	::id = id;
}

TERMINAL_COMMAND(eeprom, "Show contents of servo eeprom")
{
	servo.useId(id);
	auto & io = *terminal_io();
	io.print("===== EEPROM for Servo "); io.print(id); io.println(" =====");
	io.print("Model Number: "); io.println(servo.modelNumber());
	io.print("Firmware Version: "); io.println(servo.firmwareVersion());
	io.print("ID: "); io.println(servo.askId());
	io.print("Baud: "); io.print(servo.baud()); io.println(" bps");
	io.print("Angle Limits: ");
		auto angles = servo.angleLimit();
		io.print(angles[0] * 180 / M_PI);
		io.print(" deg - ");
		io.print(angles[1] * 180 / M_PI);
		io.println(" deg");
	io.print("Temperature Limit: "); io.print(servo.temperatureLimit()); io.println(" C");
	io.print("Voltage Limits: ");
		auto volts = servo.voltageLimit();
		io.print(volts[0]);
		io.print(" V - ");
		io.print(volts[1]);
		io.println(" V");
	io.print("Max Torque Limit: "); io.print(servo.maxTorqueLimit()); io.println(" N-dm");
	bool voltsOut, tempOut, loadOut;
	servo.outOfRangeDetected(voltsOut, tempOut, loadOut);
	io.print("Voltage out-of-range: "); io.println(voltsOut ? "alarm" : "ignore");
	io.print("Temperature out-of-range: "); io.println(tempOut ? "alarm" : "ignore");
	io.print("Torque out-of-range: "); io.println(loadOut ? "alarm" : "ignore");
}

TERMINAL_COMMAND(ram, "Show contents of servo ram")
{
	servo.useId(id);
	auto & io = *terminal_io();
	io.print("===== RAM for Servo "); io.print(id); io.println(" =====");
	io.print("Torque Enabled: "); io.println(servo.activated() ? "enabled" : "disabled");
	io.print("LED: -");
		auto color = servo.led();
		if (color.red) io.print("red-");
		if (color.green) io.print("green-");
		if (color.blue) io.print("blue-");
		io.println();
	io.print("PID: ");
		auto pid = servo.pidGain();
		io.print(pid[0]); io.print(" ");
		io.print(pid[1]); io.print(" ");
		io.print(pid[2]); io.println();
	io.print("Goal Position: "); io.print(servo.whereGoal() * 180 / M_PI); io.println(" deg");
	io.print("Goal Speed: "); io.print(servo.goalSpeed()); io.println(" rad/sec");
	io.print("Torque Limit: "); io.print(servo.torqueLimit()); io.println(" N-dm");
	io.print("Position: "); io.print(servo.where() * 180 / M_PI); io.println(" deg");
	io.print("Speed: "); io.print(servo.speed()); io.println(" rad/sec");
	io.print("Torque: "); io.print(servo.torque()); io.println(" N-dm");
	io.print("Voltage: "); io.print(servo.voltage()); io.println(" V");
	io.print("Temperature: "); io.print(servo.temperature()); io.println(" C");
	io.print("Moving: "); io.println(servo.moving() ? "in motion" : "still");
	bool voltsGood, tempGood, loadGood, allGood;
	allGood = servo.stateInRange(voltsGood, tempGood, loadGood);
	io.print("Voltage status: "); io.println(voltsGood ? "good" : "ALARM");
	io.print("Temperature status: "); io.println(tempGood ? "good" : "ALARM");
	io.print("Torque status: "); io.println(loadGood ? "good" : "ALARM");
	io.print("Servo status: "); io.println(allGood ? "good" : "ALARM");
	io.print("Punch: "); io.print(servo.punch() * 100.0); io.println("%");
}

TERMINAL_COMMAND(position, "Move servo")
{
	if (argc != 1) {
		terminal_io()->println("Usage: position degrees");
		return;
	}
	double degrees = atof(argv[0]);
	servo.useId(id);
	servo.activate();
	servo.go(degrees * M_PI / 180.0);
}

TERMINAL_COMMAND(led, "LED color")
{
	if (argc != 1) {
		terminal_io()->println("Usage: led integer");
		return;
	}
	int led = atoi(argv[0]);
	servo.useId(id);
	servo.led({led & 1, led & 2, led & 4});
}

/**
 * Setup function
 */
void setup()
{
    terminal_init(&SerialUSB);
    ServoRhobanXL320::useBaud(1000000);
    int firstServo = -1;
    numServos = 0;
    for (unsigned int i = 0; i < 253; ++ i) {
	    servo.useId(i);
	    if (servo.ping()) {
		    if (firstServo == -1) firstServo = i;
		    terminal_io()->print("Found servo "); terminal_io()->println(i);
		    ++ numServos;
	    }
    }
    if (firstServo == -1) {
	    id = ServoRhobanXL320::broadcast().id();
	} else {
		id = firstServo;
	}
}

/**
 * Loop function
 */
void loop()
{
    terminal_tick();
    ServoRhobanXL320::tick();
}

void tick()
{
	loop();
}
