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

//private:
	LegSymmetric back;
	LegSymmetric left;
	LegSymmetric right;
};


void setup()
{
	//Serial.begin(9600);
	ThreeLeggedPrototype prototype;

	//prototype.compactPose();

	//prototype.buildingPose();
	//prototype.standingPose(0.75);
	//return;
	
	
	prototype.back.setAimFromA(M_PI_2);
	for (int i = 11; i < 20; ++ i)
		pinMode(i, INPUT);
	bool rotary = analogRead(4) < 768;
	int mode = 0;
	unsigned long startTime = millis();
	for (;;) {
		//Serial.println(analogRead(4));
		bool nextRotary = analogRead(4) < 768;
		if (rotary != nextRotary && millis() - startTime > 512) {
			rotary = nextRotary;
			mode = (mode + 1) % 3;
			startTime = millis();
		}
		switch(mode) {
		case 0:
			prototype.standingPose(0.125, M_PI_4);
			break;
		case 1:
			prototype.standingPose(-cos((millis() - startTime) / 1024.0) * (5.0/16.0) + (7.0/16.0));
			break;
		case 2:
			//prototype.back.
			break;
		}
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
