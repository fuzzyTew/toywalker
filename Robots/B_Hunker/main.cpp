#include <ToyWalker.h>

#include "kinematics_ikfast.backLeftFoot.translation3d.hpp"
#include "kinematics_ikfast.backRightFoot.translation3d.hpp"
#include "kinematics_ikfast.frontLeftFoot.translation3d.hpp"
#include "kinematics_ikfast.frontRightFoot.translation3d.hpp"

#include <Walker.hpp>
#include <LimbIkFast.hpp>
#include <ServoRhobanXL320.hpp>

#include <stdlib.h>
#include <wirish/wirish.h>
#include <terminal.h>
#include <main.h>

using namespace toywalker;

class Hunker
{
public:
	Hunker()
	: backRightAimingHip(2), backRightLiftingHip(1), backRightKnee(3),
	  backLeftAimingHip(4), backLeftLiftingHip(5), backLeftKnee(6),
	  frontRightAimingHip(7), frontRightLiftingHip(8), frontRightKnee(9),
	  frontLeftAimingHip(10), frontLeftLiftingHip(11), frontLeftKnee(12),
	  ikSelector({ {1, 1}, {2, -1} }),
	  backRightLeg({-0.72, 0, 0.15},
	               {7 * 0.06, 11.5 * 0.06},
		       kinematics_ikfast_backRightFoot_translation3d::ComputeIk,
		       kinematics_ikfast_backRightFoot_translation3d::ComputeFk,
		       ikSelector,
		       backRightAimingHip, backRightLiftingHip, backRightKnee),
	  backLeftLeg({0, 0.72, 0.15},
	              {7 * 0.06, 11.5 * 0.06},
		      kinematics_ikfast_backLeftFoot_translation3d::ComputeIk,
		      kinematics_ikfast_backLeftFoot_translation3d::ComputeFk,
		      ikSelector,
		      backLeftAimingHip, backLeftLiftingHip, backLeftKnee),
	  frontRightLeg({0, -0.72, 0.15},
	                {7 * 0.06, 11.5 * 0.06},
		        kinematics_ikfast_frontRightFoot_translation3d::ComputeIk,
		        kinematics_ikfast_frontRightFoot_translation3d::ComputeFk,
			ikSelector,
		        frontRightAimingHip, frontRightLiftingHip, frontRightKnee),
	  frontLeftLeg({0.72, 0, 0.15},
	               {7 * 0.06, 11.5 * 0.06},
		       kinematics_ikfast_frontLeftFoot_translation3d::ComputeIk,
		       kinematics_ikfast_frontLeftFoot_translation3d::ComputeFk,
		       ikSelector,
		       frontLeftAimingHip, frontLeftLiftingHip, frontLeftKnee),
	  walker({0, 0, 0},
	         backRightLeg, backLeftLeg, frontRightLeg, frontLeftLeg)
	{ } 

	void tick()
	{
    		ServoRhobanXL320::tick();

		walker.heightPct(0.5 + sin(millis() / 1024.0) / 2.125);
		//backRightLeg.goal(backRightLeg.hip() + Eigen::Vector3d(0.36, 0, -backRightLeg.reach() / 4));
		//backRightLeg.goal({-0.72, 0, -0.4 + sin(millis() / 1024.0) / 32});
		//backLeftLeg.goal({0, 0.72, -0.4 + sin(millis() / 1024.0) / 32});
		//frontRightLeg.goal({0, -0.72, -0.4 + sin(millis() / 1024.0) / 32});
		//frontLeftLeg.goal({0.72, 0, -0.4 + sin(millis() / 1024.0) / 32});
	}

	ServoRhobanXL320 backRightAimingHip, backRightLiftingHip, backRightKnee;
	ServoRhobanXL320 backLeftAimingHip, backLeftLiftingHip, backLeftKnee;
	ServoRhobanXL320 frontRightAimingHip, frontRightLiftingHip, frontRightKnee;
	ServoRhobanXL320 frontLeftAimingHip, frontLeftLiftingHip, frontLeftKnee;

	IKSolutionNatural ikSelector;
	
	LimbIkFast backRightLeg;
	LimbIkFast backLeftLeg;
	LimbIkFast frontRightLeg;
	LimbIkFast frontLeftLeg;

	Walker walker;
};

Hunker * hunker;

TERMINAL_COMMAND(heightpct, "Set height of robot from 0.0 - 1.0")
{
	if (argc != 1) {
		terminal_io()->println("Usage: heightpct pct");
		return;
	}
	hunker->walker.heightPct(atof(argv[0]));
}

TERMINAL_COMMAND(height, "Set height of robot in decimeters")
{
	if (argc != 1) {
		terminal_io()->println("Usage: heightpct dm");
		return;
	}
	hunker->walker.height(atof(argv[0]));
}

TERMINAL_COMMAND(heightrange, "Get heights robot can reach")
{
	auto range = hunker->walker.heightRange();
	terminal_io()->print(range[0]);
	terminal_io()->print(" dm - ");
	terminal_io()->print(range[1]);
	terminal_io()->println(" dm");
}


TERMINAL_COMMAND(go, "GO")
{
	auto a = hunker->frontRightLeg.angles();
	terminal_io()->print("angles(): ");
	for (size_t i = 0; i < a.size(); ++ i) {
		if (i)
			terminal_io()->print(", ");
		terminal_io()->print(a[i]);
	}
	terminal_io()->println();
	a = hunker->frontRightLeg.anglesGoal();
	terminal_io()->print("anglesGoal(): ");
	for (size_t i = 0; i < a.size(); ++ i) {
		if (i)
			terminal_io()->print(", ");
		terminal_io()->print(a[i]);
	}
	terminal_io()->println();
	Eigen::Vector3d pos{0, -0.72, -0.4 + sin(millis() / 1024.0) / 32};
	terminal_io()->print("pos: ");
	for (size_t i = 0; i < pos.size(); ++ i) {
		if (i)
			terminal_io()->print(", ");
		terminal_io()->print(pos[i]);
	}
	terminal_io()->println();
	bool success = hunker->frontRightLeg.goal(pos);
	terminal_io()->println(success ? "success" : "failure");
	a = hunker->frontRightLeg.anglesGoal();
	terminal_io()->print("anglesGoal(): ");
	for (size_t i = 0; i < a.size(); ++ i) {
		if (i)
			terminal_io()->print(", ");
		terminal_io()->print(a[i]);
	}
	terminal_io()->println();
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
