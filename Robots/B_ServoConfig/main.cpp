#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <main.h>

// TODO: remove
#include <dxl.h>

#include <ToyWalker.h>
#include <ServoRhobanXL320.hpp>

using namespace toywalker;

unsigned numServos = 0;
ServoRhobanXL320 servos[12];
bool paused = false;
uint32 startMillis = 0;
unsigned int outputServo = 0;

TERMINAL_COMMAND(setid, "Changes id of a servo")
{
	if (numServos != 1) {
		terminal_io()->println("ERR: Require exactly 1 connected servo.");
		return;
	}
	if (argc != 1) {
		terminal_io()->println("Usage: setid id");
		return;
	}
	servos[0].idBroadcast();
	servos[0].idSet(atoi(argv[0]));
}

TERMINAL_COMMAND(eeprom, "Show contents of servo eeprom")
{
	auto & io = *terminal_io();
	if (argc != 1) {
		io.println("Usage: eeprom id");
		return;
	}
	ServoRhobanXL320 servo(atoi(argv[0]));
	io.print("===== EEPROM for Servo "); io.print(servo.id()); io.println(" =====");
	io.print("Model Number: "); io.println(servo.modelNumber());
	io.print("Firmware Version: "); io.println(servo.firmwareVersion());
	io.print("ID: "); io.println(servo.idAsk());
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
	io.print("Max Torque Limit: "); io.print(servo.torqueLimitMax()); io.println(" N-dm");
	bool voltsOut, tempOut, loadOut;
	servo.limitsAlarmed(voltsOut, tempOut, loadOut);
	io.print("Voltage out-of-range: "); io.println(voltsOut ? "alarm" : "ignore");
	io.print("Temperature out-of-range: "); io.println(tempOut ? "alarm" : "ignore");
	io.print("Torque out-of-range: "); io.println(loadOut ? "alarm" : "ignore");
}

TERMINAL_COMMAND(ram, "Show contents of servo ram")
{
	auto & io = *terminal_io();
	if (argc != 1) {
		io.println("Usage: eeprom id");
		return;
	}
	ServoRhobanXL320 servo(atoi(argv[0]));
	io.print("===== RAM for Servo "); io.print(servo.id()); io.println(" =====");
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
	io.print("Goal Position: "); io.print(servo.angleGoal() * 180 / M_PI); io.println(" deg");
	io.print("Goal Speed: "); io.print(servo.velocityGoal()); io.println(" rad/sec");
	io.print("Torque Limit: "); io.print(servo.torqueLimit()); io.println(" N-dm");
	io.print("Position: "); io.print(servo.angle() * 180 / M_PI); io.println(" deg");
	io.print("Speed: "); io.print(servo.velocity()); io.println(" rad/sec");
	io.print("Torque: "); io.print(servo.torque()); io.println(" N-dm");
	io.print("Voltage: "); io.print(servo.voltage()); io.println(" V");
	io.print("Temperature: "); io.print(servo.temperature()); io.println(" C");
	io.print("Moving: "); io.println(servo.moving() ? "in motion" : "still");
	bool voltsGood, tempGood, loadGood, allGood;
	allGood = servo.limitsWithin(voltsGood, tempGood, loadGood);
	io.print("Voltage status: "); io.println(voltsGood ? "good" : "ALARM");
	io.print("Temperature status: "); io.println(tempGood ? "good" : "ALARM");
	io.print("Torque status: "); io.println(loadGood ? "good" : "ALARM");
	io.print("Servo status: "); io.println(allGood ? "good" : "ALARM");
	io.print("Punch: "); io.print(servo.punch() * 100.0); io.println("%");
}

TERMINAL_COMMAND(angle, "Move servo")
{
	if (argc != 2) {
		terminal_io()->println("Usage: angle id degrees");
		return;
	}
	ServoRhobanXL320 servo(atoi(argv[0]));
	double degrees = atof(argv[1]);
	servo.activate();
	servo.angleGoal(degrees * M_PI / 180.0);
}

TERMINAL_COMMAND(led, "LED color")
{
	if (argc != 2) {
		terminal_io()->println("Usage: led id integer");
		return;
	}
	ServoRhobanXL320 servo(atoi(argv[0]));
	int led = atoi(argv[1]);
	servo.led({led & 1, led & 2, led & 4});
}

TERMINAL_COMMAND(servos, "List connected servos")
{
	auto & io = *terminal_io();
	for (unsigned int i = 0; i < numServos; ++ i) {
		io.print(servos[i].id()); io.print(": -");
		auto color = servos[i].led();
		if (color.red) io.print("red-");
		if (color.green) io.print("green-");
		if (color.blue) io.print("blue-");
		io.print(" ");
		io.print(servos[i].angle() * 180 / M_PI); io.println(" deg");
	}
}

TERMINAL_COMMAND(scan, "Rebuild servo list")
{
	auto & io = *terminal_io();

    numServos = 0;
    for (unsigned int i = 0; i < 253 && numServos < sizeof(servos) / sizeof(servos[0]); ++ i) {
	    servos[numServos].idUse(i);
	    if (servos[numServos].present()) {
		    servos[numServos].activate();
		    servos[numServos].led({1,1,0});
		    io.print("Found servo "); io.println(i);
		    ++ numServos;
	    }
    }
}

TERMINAL_COMMAND(pause, "Stop automatic motion")
{
	paused = true;
}

TERMINAL_COMMAND(resmue, "Resmue automatic motion")
{
	paused = false;
}

/**
 * Setup function
 */
void setup()
{
    terminal_init(&SerialUSB);
	unsigned int i;

	//for (i = 0; i < 10; ++ i)
	//	pinMode(i, INPUT_ANALOG);
	pinMode(0, INPUT_ANALOG);
	pinMode(1, INPUT_ANALOG);
	pinMode(10, INPUT);

    ServoRhobanXL320::baudUse(1000000);

    numServos = 0;
    for (i = 0; i < 253 && numServos < sizeof(servos) / sizeof(servos[0]); ++ i) {
	    servos[numServos].idUse(i);
	    if (servos[numServos].present()) {
		    servos[numServos].activate();
		    servos[numServos].led({1,1,0});
		    ++ numServos;
	    }
    }
}

int mode = 0;

/**
 * Loop function
 */
void loop()
{
    terminal_tick();
    ServoRhobanXL320::tick();

    if (numServos) {
	    uint32 time = millis() - startMillis;
	    if (time < servos[outputServo].id() * 600) {
		    digitalWrite(BOARD_LED_PIN, (time % 600) < 300);
	    } else if (time < servos[outputServo].id() * 600 + 600) {
		    digitalWrite(BOARD_LED_PIN, HIGH);
	    } else {
		    startMillis = millis();
		    if (!paused) {
				outputServo = (outputServo + 1) % numServos;
				mode = (mode + 1) % 4;
			switch (mode) {
			case 0:
				ServoRhobanXL320::broadcast().angleGoal(-M_PI_2);
				break;
			case 1:
				ServoRhobanXL320::broadcast().angleGoal(0);
				break;
			case 2:
				ServoRhobanXL320::broadcast().angleGoal(M_PI_2);
				break;
			case 3:
				ServoRhobanXL320::broadcast().angleGoal(0);
				break;
			}
		}

			/*
		    unsigned int oldNumServos = numServos;
		    numServos = 0;

		    for (i = 0; i < 12; ++ i) {
			    servo.idUse(i);
			    if (servo.present()) {
				    servos[numServos].idUse(i);
				    servos[numServos].activate();
				    ++ numServos;
			    }
		    }
		
		    if (numServos != oldNumServos) {
				terminal_io()->print(numServos); terminal_io()->println(" servo(s)");
			    startMillis = millis();
			    outputServo = 0;
		    }
		    */
	    }
	}
}

void tick()
{
	loop();
}
