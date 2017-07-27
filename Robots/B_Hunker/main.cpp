#include <ToyWalker.h>

#include "kinematics_ikfast.backLeftFoot.translation3d.hpp"
#include "kinematics_ikfast.backRightFoot.translation3d.hpp"
#include "kinematics_ikfast.frontLeftFoot.translation3d.hpp"
#include "kinematics_ikfast.frontRightFoot.translation3d.hpp"

#include <Walker.hpp>
#include <WalkGaitFormula.hpp>
#include <WalkStepCompromise.hpp>
#include <WalkStepCubic.hpp>
#include <PathSegmentQuadratic.hpp>
#include <PathSequence.hpp>
#include <PathSegmentLine.hpp>
#include <PathEllipse.hpp>
#include <PathDelayed.hpp>
#include <PathLookAt.hpp>
#include <PathConstant.hpp>
#include <PathProduct.hpp>
#include <GroundPlane.hpp>
#include <LimbIkFast.hpp>
#include <ServoRhobanXL320.hpp>

#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <main.h>

using namespace toywalker;

class Hunker : public Walker
{
public:
	Hunker()
	: Walker({0,0,0}, ground, gait, 3 * 0.06),
	  gait(2*0.75, {{0, 0, 0.625*0.75}, {3, 0.375*0.75, 1.0*0.75}, {1, 1.0*0.75, 1.625*0.75}, {2, 1.375*0.75, 2.0*0.75}}),
	  backRightAimingHip(2), backRightLiftingHip(1), backRightKnee(3),
	  backLeftAimingHip(4), backLeftLiftingHip(5), backLeftKnee(6),
	  frontRightAimingHip(7), frontRightLiftingHip(8), frontRightKnee(9),
	  frontLeftAimingHip(10), frontLeftLiftingHip(11), frontLeftKnee(12),
	  ikSelector({ {1, 1}, {2, -1} }),
	  backRightLeg(*this,
	               {-0.72, 0, 0.15},
	               {7 * 0.06, 10 * 0.06},
		       kinematics_ikfast_backRightFoot_translation3d::ComputeIk,
		       kinematics_ikfast_backRightFoot_translation3d::ComputeFk,
		       ikSelector,
		       backRightAimingHip, backRightLiftingHip, backRightKnee),
	  backLeftLeg(*this,
	              {0, 0.72, 0.15},
	              {7 * 0.06, 10 * 0.06},
		      kinematics_ikfast_backLeftFoot_translation3d::ComputeIk,
		      kinematics_ikfast_backLeftFoot_translation3d::ComputeFk,
		      ikSelector,
		      backLeftAimingHip, backLeftLiftingHip, backLeftKnee),
	  frontRightLeg(*this,
	                {0, -0.72, 0.15},
	                {7 * 0.06, 10 * 0.06},
		        kinematics_ikfast_frontRightFoot_translation3d::ComputeIk,
		        kinematics_ikfast_frontRightFoot_translation3d::ComputeFk,
			ikSelector,
		        frontRightAimingHip, frontRightLiftingHip, frontRightKnee), 
	  frontLeftLeg(*this,
	               {0.72, 0, 0.15},
	               {7 * 0.06, 10 * 0.06},
		       kinematics_ikfast_frontLeftFoot_translation3d::ComputeIk,
		       kinematics_ikfast_frontLeftFoot_translation3d::ComputeFk,
		       ikSelector,
		       frontLeftAimingHip, frontLeftLiftingHip, frontLeftKnee),
	  backRightStep(*this, backRightLeg, 3/16.0),
	  backLeftStep(*this, backLeftLeg, 3/16.0),
	  frontRightStep(*this, frontRightLeg, 3/16.0),
	  frontLeftStep(*this, frontLeftLeg, 3/16.0),
	  subpath1({0,0,0}, {4,0,0}, 12),
	  subpath2({4,0,0}, {0,1,0}, {0,0,0}, 48),
	  movePath(subpath1, subpath2),
	  lookPath(movePath, -6),
	  bodyPath(movePath, lookPath, {0,0,1}),
	  bodyAdjustment(Isometry3(AngleAxis(M_PI / 4, Vector3::UnitZ()))),
	  adjustedPath(bodyPath, bodyAdjustment),
	  going(true)
	{
		curlimb = limbs() - 1;
		state = 2;
		lastTime = millis() - 1000;
		fpsTime = millis();
		fpsCount = 0;
		fpsTotal = 0;
		fpsStart = fpsTime;

		auto hr = heightRange(Isometry3::Identity());
		Real height = hr[0] + 1/16.0;//(hr[0] + hr[1]) / 2;
		Vector3 x = subpath1.beginning();
		x.z() = height;
		subpath1.beginning(x);
		x = subpath1.end();
		x.z() = height;
		subpath1.end(x);
		x = subpath2.center();
		x.z() = height;
		subpath2.center(x);

		path(adjustedPath);
	} 

	void tick()
	{
    		ServoRhobanXL320::tick();
		Walker::tick();
		/*
		Real time = path().now();
		Vector3 pos = path().at(time).translation();
		terminal_io()->print(time); terminal_io()->print(": ");
		terminal_io()->print(pos.x()); terminal_io()->print(" ,");
		terminal_io()->print(pos.y()); terminal_io()->print(" ,");
		terminal_io()->print(pos.z()); terminal_io()->println();
		*/


#if 0
		if (going) {
			/*++ fpsCount; ++ fpsTotal;
			if (millis() - fpsTime >= 500) {
				terminal_io()->print("FPS: "); terminal_io()->print(fpsCount * 1000.0 / (millis() - fpsTime));
				terminal_io()->print(" AVG: "); terminal_io()->print(fpsTotal * 1000.0 / (millis() - fpsStart));
				terminal_io()->println();
				fpsTime = millis();
				fpsCount = 0;
			}*/
			//auto test = limb(0).footGoalArea();//(ground.projection(bodyToArea() * limb(0).homeBody()));
			//terminal_io()->print("Test = ");
			//for (size_t i = 0; i < test.size(); ++ i) {
			//	if (i)
			//		terminal_io()->print(", ");
			//	terminal_io()->print(test(i));
			//}
			//terminal_io()->println();
			//limb(0).goalArea(test);
			/*uint32 time = millis();
			switch(state) {
			case 0: // moving to home
				if (time - lastTime >= 1000) {
				}
				break;
			case 1: // raising leg
			case 2: // lowering leg
				if (time - lastTime < 1000) {
				} else {
					limb.
					state = 0;
				}
				break;
			}*/
			///*
			Real scale1 = 512;//sin(millis() / 32768) * 1024.0 + 1024.0 + 256.0;
			Real scale2 = 512;//cos(millis() / 31000) * 1024.0 + 1024.0 + 256.0;
			Real scale3 = 512;//-sin(millis() / 30000) * 1024.0 + 1024.0 + 256.0;
			Real scale4 = 512;//-cos(millis() / 29000) * 1024.0 + 1024.0 + 256.0;
			//Real time = fmod(millis(), 2048);
			//if (time < 1024)
			//	walker.heightPct(time / 1024.0);
			//else
			//	walker.heightPct((2048 - time) / 1024.0);
			//heightPct(0.5 + sin(millis() / 1024.0) / 2.125);
			auto range = heightRange();
			//walker.position({sin(millis() / 1024.0) * 0.25, cos(millis() / 1024.0) * 0.25, (range[1] - range[0])*((sin(millis() / 512.0) * 0.375 + 0.375)) + range[0]});
			bodyToArea(
				Translation3(
					sin(millis() / scale1) * 2.0/16,
					cos(millis() / scale1) * 2.0/16,
					(range[1] - range[0]) * ((sin(millis() / scale2) * 5.0/16 + -20.0/16)) + range[0]
				)  * 
				AngleAxis(sin(millis()/scale3)*0.5, Vector3::UnitZ())  *
				AngleAxis(sin(millis()/scale4)*3.0/16, Vector3::UnitX()) *
				AngleAxis(cos(millis()/scale4)*3.0/16, Vector3::UnitY())//*/
			);
			//
		}
		//backRightLeg.goal(backRightLeg.hip() + Vector3(0.36, 0, -backRightLeg.reach() / 4));
		//backRightLeg.goal({-0.72, 0, -0.4 + sin(millis() / 1024.0) / 32});
		//backLeftLeg.goal({0, 0.72, -0.4 + sin(millis() / 1024.0) / 32});
		//frontRightLeg.goal({0, -0.72, -0.4 + sin(millis() / 1024.0) / 32});
		//frontLeftLeg.goal({0.72, 0, -0.4 + sin(millis() / 1024.0) / 32});
#endif
	}

	GroundPlane ground;

	WalkGaitFormula gait;

	ServoRhobanXL320 backRightAimingHip, backRightLiftingHip, backRightKnee;
	ServoRhobanXL320 backLeftAimingHip, backLeftLiftingHip, backLeftKnee;
	ServoRhobanXL320 frontRightAimingHip, frontRightLiftingHip, frontRightKnee;
	ServoRhobanXL320 frontLeftAimingHip, frontLeftLiftingHip, frontLeftKnee;

	IKSolutionNatural ikSelector;
	
	LimbIkFast backRightLeg;
	LimbIkFast backLeftLeg;
	LimbIkFast frontRightLeg;
	LimbIkFast frontLeftLeg;

	WalkStepCubic backRightStep;
	WalkStepCubic backLeftStep;
	WalkStepCubic frontRightStep;
	WalkStepCubic frontLeftStep;

	PathSegmentLine<Vector3> subpath1;
	PathEllipse subpath2;
	PathSequence<Vector3> movePath;
	PathDelayed<Vector3> lookPath;
	PathLookAt bodyPath;

	PathConstant<Isometry3> bodyAdjustment;
	PathProduct<Isometry3, 2> adjustedPath;


	bool going;
	size_t curlimb;
	unsigned int state;
	uint32 lastTime;
	Vector3 lastPos;
	Vector3 nextPos;

	uint32 fpsTime, fpsStart;
	size_t fpsCount, fpsTotal;

};

Hunker * hunker;

TERMINAL_COMMAND(project, "Project legs to ground to stand")
{
	for (size_t i = 0; i < hunker->limbs(); ++ i) {
		hunker->limb(i).goalArea(hunker->ground.projection(hunker->bodyToArea() * hunker->limb(i).homeBody()));
	}
}

TERMINAL_COMMAND(attach, "Affix legs to ground")
{
	for (size_t i = 0; i < hunker->limbs(); ++ i) {
		hunker->limb(i).attach();
	}
}

TERMINAL_COMMAND(detach, "Release legs from ground")
{
	for (size_t i = 0; i < hunker->limbs(); ++ i) {
		hunker->limb(i).detach();
	}
}

/*
TERMINAL_COMMAND(heightpct, "Set height of robot from 0.0 - 1.0")
{
	if (argc != 1) {
		terminal_io()->println("Usage: heightpct pct");
		return;
	}
	hunker->heightPct(atof(argv[0]));
}
*/

/*
TERMINAL_COMMAND(height, "Set height of robot in decimeters")
{
	if (argc != 1) {
		terminal_io()->println("Usage: heightpct dm");
		return;
	}
	hunker->height(atof(argv[0]));
}
*/

/*
TERMINAL_COMMAND(heightrange, "Get heights robot can reach")
{
	auto range = hunker->heightRange();
	terminal_io()->print(range[0]);
	terminal_io()->print(" dm - ");
	terminal_io()->print(range[1]);
	terminal_io()->println(" dm");
}
*/


/*
TERMINAL_COMMAND(go, "Continuous motion")
{
	hunker->going = true;
}

TERMINAL_COMMAND(stop, "Halt motion")
{
	hunker->going = false;
}

TERMINAL_COMMAND(activate, "Power to servos")
{
	hunker->backRightLeg.activate();
	hunker->backLeftLeg.activate();
	hunker->frontRightLeg.activate();
	hunker->frontLeftLeg.activate();
}

TERMINAL_COMMAND(deactivate, "Loose limbs")
{
	hunker->backRightLeg.deactivate();
	hunker->backLeftLeg.deactivate();
	hunker->frontRightLeg.deactivate();
	hunker->frontLeftLeg.deactivate();
}
*/

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


/**
 * Setup function
 */
void setup()
{
    terminal_init(&SerialUSB);
	ServoRhobanXL320::baudUse(1000000);

	Hunker hunker;
	::hunker = &hunker;

    for (;;) {
    	terminal_tick();
	hunker.tick();

	//hunker.backRightAimingHip.activate();
	//hunker.backRightAimingHip.angleGoal(-M_PI/4);
	//hunker.backLeftLeg.servo(0).angleGoal(M_PI/4);
    }
}

/**
 * Loop function
 */
void loop()
{
}
