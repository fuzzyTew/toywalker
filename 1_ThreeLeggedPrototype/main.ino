#include "../lib/ServoPermanentHextronik.hpp"


extern void setup() {
	ServoPermanentHextronik<2> shoulder1;
	ServoPermanentHextronik<5> shoulder2;
	ServoPermanentHextronik<8> shoulder3;
	ServoPermanentHextronik<3> hip1;
	ServoPermanentHextronik<6> hip2;
	ServoPermanentHextronik<9> hip3;
	ServoPermanentHextronik<4> knee1;
	ServoPermanentHextronik<7> knee2;
	ServoPermanentHextronik<10> knee3;

	shoulder1.goRatio(0.5);
	shoulder2.goRatio(0.5);
	shoulder3.goRatio(0.5);
	hip1.goRatio(0.5);
	hip2.goRatio(0.5);
	hip3.goRatio(0.5);
	knee1.goRatio(0.5);
	knee2.goRatio(0.5);
	knee3.goRatio(0.5);
}

extern void loop()
{
}
