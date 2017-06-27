#pragma once

class ServoImplementation
{
public:
	virtual float goRatio(float ratio) = 0;
	virtual float radiansPerRatio() = 0;
	float go(float radians)
	{
		float radsPerRat = radiansPerRatio();
		return goRatio(radians / radsPerRat) * radsPerRat;
	}
};
