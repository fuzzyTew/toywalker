#include <ToyWalker.h>

#include <ServoPermanentHextronik.hpp>
#include <Joint.hpp>
#include <LegSymmetric.hpp>

class ThreeLeggedPrototype
{
public:
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

	void setLift(float lift)
	{
		back.setLift(lift);
		left.setLift(lift);
		right.setLift(lift);
	}

	void setLiftFront(float lift)
	{
		left.setLift(lift);
		right.setLift(lift);
	}

	void setKnee(float knee)
	{
		back.setKnee(knee);
		left.setKnee(knee);
		right.setKnee(knee);
	}

	void setAimFront(float aim)
	{
		left.setAimFromA(aim);
		right.setAimFromA(aim);
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

	void standingPose(float heightAmount = 0.5, float lift = 0)
	{
		back.setAimFromA(M_PI_2);
		setAimFront(M_PI_2 + 0.25);
		//setLiftFront(0);
		//back.setLift(0.5);
		setLift(lift);
		setKnee(heightAmount * M_PI);
	}

	void setServo(int index, float position)
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
	LegSymmetric right;
};


void setup()
{
	Serial.begin(9600);
	ThreeLeggedPrototype prototype;
	
	//for (int i = 11; i < 20; ++ i)
	//	pinMode(i, INPUT);
	pinMode(3, INPUT_PULLUP);
	digitalWrite(3, HIGH);
	bool rotary = analogRead(3);
	int mode = 3;
	unsigned long startTime = millis();
	unsigned long lastTime = startTime;
	int testServo = 0;
	float testPosition = 0.5;
	int testState = 0;
	for (;;) {
		unsigned long time = millis();
		Serial.println(analogRead(3));
		bool nextRotary = analogRead(3);
		if (rotary != nextRotary && time - startTime > 512) {
			rotary = nextRotary;
			mode = (mode + 1) % 5;
			startTime = millis();
		}
		switch(mode) {
		case 0:
			// hug ground in such a way we are very likely to fall on our feet
			prototype.standingPose(0.125, M_PI_4);
			break;
		case 1:
			// squats
			prototype.standingPose(-cos((millis() - startTime) / 1024.0) * (5.0/16.0) + (7.0/16.0));
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

				float delta = (time - lastTime) / 1024.0;
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
		}
		lastTime = time;
		/*
		prototype.back.setKnee(M_PI * (digitalRead(11) ? 0.75 : 0.25));
		prototype.left.setKnee(M_PI * (digitalRead(12) ? 0.75 : 0.25));
		prototype.right.setKnee(M_PI * (digitalRead(13) ? 0.75 : 0.25));

		prototype.back.setAimFromA(M_PI_4 + M_PI_2 * analogRead(0) / 1023.0);
		prototype.back.setLift(-M_PI * 1.0/8.0 + M_PI_2 * analogRead(1) / 1023.0);

		prototype.left.setLift(-M_PI * 1.0/8.0 + M_PI_4 * analogRead(2) / 1023.0);
		prototype.left.setAimFromA(M_PI * 3.0/8.0 + M_PI_4 * analogRead(3) / 1023.0);
		prototype.right.setLift(-M_PI * 1.0/8.0 + M_PI_4 * analogRead(4) / 1023.0);
		prototype.right.setAimFromA(M_PI * 3.0/8.0 + M_PI_4 * analogRead(5) / 1023.0);
		*/
	}
	//*/
} 

void loop()
{
}
