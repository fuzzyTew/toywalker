#include <SoftwareSerial.h>
#include <ToyWalker.h>

#include <LimbIkFast.hpp>
#include <ServoPermanentHextronik.hpp>
//#include <Joint.hpp>
//#include <LegSymmetric.hpp>

#include "kinematics_ikfast.back_foot.translation3d.hpp"
#include "kinematics_ikfast.left_foot.translation3d.hpp"
#include "kinematics_ikfast.right_foot.translation3d.hpp"

#include "../PrototypeRemote/PrototypeRemote.hpp"

class ThreeLeggedPrototype
{
public:
	void backFootGo(double xPct, double yPct, double zPct)
	{
		backLeg.go({kinematics_ikfast_back_foot_translation3d::xMin + xPct, kinematics_ikfast_back_foot_translation3d::yMin + yPct, kinematics_ikfast_back_foot_translation3d::zMin + zPct});
	}

	void leftFootGo(double xPct, double yPct, double zPct)
	{
		leftLeg.go({kinematics_ikfast_left_foot_translation3d::xMin + xPct, kinematics_ikfast_left_foot_translation3d::yMin + yPct, kinematics_ikfast_left_foot_translation3d::zMin + zPct});
	}

	void rightFootGo(double xPct, double yPct, double zPct)
	{
		rightLeg.go({kinematics_ikfast_right_foot_translation3d::xMin + xPct, kinematics_ikfast_right_foot_translation3d::yMin + yPct, kinematics_ikfast_right_foot_translation3d::zMin + zPct});
	}
	
//private:
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

	
/*
	ThreeLeggedPrototype()
	: back(Joint(new ServoPermanentHextronik<2>(), 0, M_PI),
	       Joint(new ServoPermanentHextronik<4>(), 0, M_PI),
	       Joint(new ServoPermanentHextronik<3>(), M_PI - M_PI_4, 0),
	       M_PI_4),
	  left(Joint(new ServoPermanentHextronik<5>(), M_PI, 0),
	       Joint(new ServoPermanentHextronik<6>(), 0, M_PI),
	       Joint(new ServoPermanentHextronik<7>(), M_PI - M_PI_4, 0),
	       M_PI_4),
	 right(Joint(new ServoPermanentHextronik<8>(), 0, M_PI),
	       Joint(new ServoPermanentHextronik<9>(), M_PI, 0),
	       Joint(new ServoPermanentHextronik<10>(),M_PI_4, M_PI),
	       M_PI_4)
	{ }

	void setLift(double lift)
	{
		back.setLift(lift);
		left.setLift(lift);
		right.setLift(lift);
	}

	void setLiftFront(double lift)
	{
		left.setLift(lift);
		right.setLift(lift);
	}

	void setKnee(double knee)
	{
		back.setKnee(knee);
		left.setKnee(knee);
		right.setKnee(knee);
	}

	void setAimFront(double aim)
	{
		left.setAimFromA(aim);
		right.setAimFromA(aim);
	}

	void setKneeFront(double knee)
	{
		left.setKnee(knee);
		right.setKnee(knee);
	}

	void buildingPose()
	{
		setLift(M_PI_4);
		setKnee(M_PI_2);

		setAimFront(M_PI_2);
		back.setAimFromA(M_PI_2);
	}

	void compactPose()
	{
		setLift(M_PI);
		setKnee(0);

		setAimFront(M_PI_2);
		back.setAimFromA(M_PI_2);
	}

	void standingPose(double heightAmount = 0.5, double lift = 0)
	{
		back.setAimFromA(M_PI_2);
		setAimFront(M_PI_2 + 4.0/16.0);
		setLiftFront(lift);
		back.setLift(lift + 0.25);
		//setLift(lift);
		setKneeFront(heightAmount * M_PI);
		//if (heightAmount > 0.125 + 1.0/8.0) {
		//	heightAmount -= 1.0/8.0;
		//}
		back.setKnee(heightAmount * M_PI);
	}

	void setServo(int index, double position)
	{
		Joint * j;
		switch(index) {
		case 0: j = &back.getAim(); break;
		case 1: j = &back.getLift(); break;
		case 2: j = &back.getKnee(); break;
		case 3: j = &left.getAim(); break;
		case 4: j = &left.getLift(); break;
		case 5: j = &left.getKnee();  break;
		case 6: j = &right.getAim(); break;
		case 7: j = &right.getLift(); break;
		case 8: j = &right.getKnee(); break;
		default: return;
		}

		j->getServo().goRatio(position);
	}

//private:
	LegSymmetric back;
	LegSymmetric left;
	LegSymmetric right
*/
};

void setup()
{
	ThreeLeggedPrototype prototype;

	Serial.begin(PrototypeRemote::BAUD);
	PrototypeRemote remote(Serial);

	
	//std::tr1::get<0>(prototype.leftLeg.servos).go(1.1);
	//std::tr1::get<0>(prototype.rightLeg.servos).go(1.1);
	//return;

	double x = 0.5, y = 0.5, z = 0.5;

	for (;;) {
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
			std::tr1::get<0>(prototype.backLeg.servos).go(0);
			std::tr1::get<1>(prototype.backLeg.servos).go(0);
			std::tr1::get<2>(prototype.backLeg.servos).go(0);
			std::tr1::get<0>(prototype.leftLeg.servos).go(0);
			std::tr1::get<1>(prototype.leftLeg.servos).go(0);
			std::tr1::get<2>(prototype.leftLeg.servos).go(0);
			std::tr1::get<0>(prototype.rightLeg.servos).go(0);
			std::tr1::get<1>(prototype.rightLeg.servos).go(0);
			std::tr1::get<2>(prototype.rightLeg.servos).go(0);
			break;
		}
	}
/*

	unsigned long lastTime = remote.modeMillis();
	int testServo = 0;
	double testPosition = 0.5;
	int testState = 0;
	for (;;) {
		remote.receive();
		Serial.println(remote.mode());
		unsigned long time = millis();
		switch(remote.mode()) {
		case 0:
			// hug ground in such a way we are very likely to fall on our feet
			prototype.standingPose(0.125, M_PI_4);
			break;
		case 1:
			// squats
			prototype.standingPose(-cos((millis() - remote.modeMillis()) / 1024.0) * (5.0/16.0) + (7.0/16.0));
			break;
		case 2:
			// TODO: joystick
			//prototype.back.
			break;
		case 3:
			// "home" locations on all joints, for aligning
			prototype.buildingPose();
			break;
		case 4:
			// test range of all servos
			{
				prototype.setServo(testServo, testPosition);

				double delta = (time - lastTime) / 1024.0;
				switch (testState) {
				case 0: // forward from 0.5
					testPosition += delta;
					if (testPosition < 0.875)
						break;
					testPosition = 0.875 * 2 - testPosition + delta;
					testState = 1;
				case 1: // backward from 0.875
					testPosition -= delta;
					if (testPosition > 0.125)
						break;
					testPosition = -testPosition - delta;
					testState = 2;
				case 2: // forward from 0.125
					testPosition += delta;
					if (testPosition < 0.5)
						break;
					prototype.setServo(testServo, 0.5);
					testServo = (testServo + 1) % 9;
					testState = 0;
				}
			}
			break;
		case 5:
			prototype.compactPose();
			break;
		}
		lastTime = time;

		//prototype.back.setKnee(M_PI * (digitalRead(11) ? 0.75 : 0.25));
		//prototype.left.setKnee(M_PI * (digitalRead(12) ? 0.75 : 0.25));
		//prototype.right.setKnee(M_PI * (digitalRead(13) ? 0.75 : 0.25));

		//prototype.back.setAimFromA(M_PI_4 + M_PI_2 * analogRead(0) / 1023.0);
		//prototype.back.setLift(-M_PI * 1.0/8.0 + M_PI_2 * analogRead(1) / 1023.0);

		//prototype.left.setLift(-M_PI * 1.0/8.0 + M_PI_4 * analogRead(2) / 1023.0);
		//prototype.left.setAimFromA(M_PI * 3.0/8.0 + M_PI_4 * analogRead(3) / 1023.0);
		//prototype.right.setLift(-M_PI * 1.0/8.0 + M_PI_4 * analogRead(4) / 1023.0);
		//prototype.right.setAimFromA(M_PI * 3.0/8.0 + M_PI_4 * analogRead(5) / 1023.0);
	}
	//*/
} 

void loop()
{
}
