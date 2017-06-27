#include <ServoPermanentHextronik.hpp>

void buildingPose() {
}

class ThreeLeggedPrototype
{
public:
	void buildingPose()
	{
		shoulder1.goRatio(0.5);
		shoulder2.goRatio(0.5);
		shoulder3.goRatio(0.5);

		hip1.goRatio(0.5);
		hip2.goRatio(0.5);
		hip3.goRatio(0.5);

		// knees contracted a bit
		knee1.goRatio(0.5);
		knee2.goRatio(0.5);
		knee3.goRatio(0.5);
	}

	void standingPose()
	{
		shoulder1.goRatio(0.5);
		// front shoulders aimed forward
		shoulder2.goRatio(0.375);
		shoulder3.goRatio(0.625);

		// hips contracted a bit
		hip1.goRatio(0.375);
		hip2.goRatio(0.375);
		hip3.goRatio(0.625);

		// knees contracted a bit
		knee1.goRatio(0.625);
		knee2.goRatio(0.625);
		knee3.goRatio(0.375);
	}
private:
	ServoPermanentHextronik<2> shoulder1;
	ServoPermanentHextronik<5> shoulder2;
	ServoPermanentHextronik<8> shoulder3;
	ServoPermanentHextronik<3> hip1;
	ServoPermanentHextronik<6> hip2;
	ServoPermanentHextronik<9> hip3;
	ServoPermanentHextronik<4> knee1;
	ServoPermanentHextronik<7> knee2;
	ServoPermanentHextronik<10> knee3;
};


void setup()
{
	ThreeLeggedPrototype prototype;

	//prototype.buildingPose();
	prototype.standingPose();
} 

void loop()
{
}
