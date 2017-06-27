#include <ServoPermanentHextronik.hpp>
#include <Joint.hpp>
#include <Leg.hpp>

class ThreeLeggedPrototype
{
public:
	ThreeLeggedPrototype()
	: back(Joint(new ServoPermanentHextronik<2>(), 0, M_PI),
	       Joint(new ServoPermanentHextronik<3>(), 0, M_PI),
	       Joint(new ServoPermanentHextronik<4>(), M_PI - M_PI_4, 0),
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

	void standingPose(float heightAmount = 0.5)
	{
		back.setAimFromA(M_PI_2);
		setAimFront(M_PI_2 + 0.5);
		setLift(0);
		setKnee(heightAmount * M_PI);
	}

private:
	Leg back;
	Leg left;
	Leg right;
};


void setup()
{
	ThreeLeggedPrototype prototype;

	//prototype.buildingPose();
	//prototype.standingPose(0.75);
	for (;;) {
		prototype.standingPose(sin(millis() / 1024.0) * 0.25 + 0.5);
	}
} 

void loop()
{
}
