//#include <SoftwareSerial.h>
#include <ToyWalker.h>

#include <LimbIkFast.hpp>
#include <ServoPermanentHextronik.hpp>
#include <Walker.hpp>

#include "kinematics_ikfast.back_foot.translation3d.hpp"
#include "kinematics_ikfast.left_foot.translation3d.hpp"
#include "kinematics_ikfast.right_foot.translation3d.hpp"

#include "../PrototypeRemote/PrototypeRemote.hpp"

class ThreeLeggedPrototype : public Walker
{
public:
	ThreeLeggedPrototype()
	: Walker
	  (
		{0.5, 2.5},
		{0.5, 1.5},
		{0, 0, 0},
		backLeg, leftLeg, rightLeg
	  ),
	  backLeg({-0.75, 0.00, -0.06}),
	  leftLeg({0.3125, 0.67, -0.06}),
	  rightLeg({0.3125, -0.67, -0.06})
	{ }

	void backFootGo(double xPct, double yPct, double zPct)
	{
		limb(0).go({kinematics_ikfast_back_foot_translation3d::xMin + xPct, kinematics_ikfast_back_foot_translation3d::yMin + yPct, kinematics_ikfast_back_foot_translation3d::zMin + zPct});
	}

	void leftFootGo(double xPct, double yPct, double zPct)
	{
		limb(1).go({kinematics_ikfast_left_foot_translation3d::xMin + xPct, kinematics_ikfast_left_foot_translation3d::yMin + yPct, kinematics_ikfast_left_foot_translation3d::zMin + zPct});
	}

	void rightFootGo(double xPct, double yPct, double zPct)
	{
		limb(2).go({kinematics_ikfast_right_foot_translation3d::xMin + xPct, kinematics_ikfast_right_foot_translation3d::yMin + yPct, kinematics_ikfast_right_foot_translation3d::zMin + zPct});
	}
	
private:
	LimbIkFast <
		kinematics_ikfast_back_foot_translation3d::ComputeIk,
		kinematics_ikfast_back_foot_translation3d::ComputeFk,
		ServoPermanentHextronik<2>,
		ServoPermanentHextronik<4>,
		ServoPermanentHextronik<3>
	> backLeg;
	LimbIkFast
	<
		kinematics_ikfast_left_foot_translation3d::ComputeIk,
		kinematics_ikfast_left_foot_translation3d::ComputeFk,
		ServoPermanentHextronik<5>,
		ServoPermanentHextronik<6>,
		ServoPermanentHextronik<7>
	> leftLeg;
	LimbIkFast
	<
		kinematics_ikfast_right_foot_translation3d::ComputeIk,
		kinematics_ikfast_right_foot_translation3d::ComputeFk,
		ServoPermanentHextronik<8>,
		ServoPermanentHextronik<9>,
		ServoPermanentHextronik<10>
	> rightLeg;
};

void setup()
{
	ThreeLeggedPrototype prototype;

	Serial.begin(PrototypeRemote::BAUD);
	PrototypeRemote remote(Serial);


	double x = 0.5, y = 0.5, z = 0.5;

	for (;;) {
		unsigned int i;
		remote.receive();
		digitalWrite(13, (remote.mode() % 2) ? HIGH : LOW);
		switch(remote.mode()) {
		case 0:
			x = 1.0 - remote.joystick(0).y;
			y = remote.joystick(0).x;
			prototype.backFootGo(x, y, z);
			prototype.leftFootGo(x, y, z);
			prototype.rightFootGo(x, y, z);
			break;
		case 1:
			z = 1.0 - remote.joystick(0).y;
			prototype.backFootGo(x, y, z);
			prototype.leftFootGo(x, y, z);
			prototype.rightFootGo(x, y, z);
			break;
		case 2:
			for (i = 0; i < prototype.servos(); ++ i)
				prototype.servoGo(i, 0);
			break;
		}
	}
} 

void loop()
{
}
