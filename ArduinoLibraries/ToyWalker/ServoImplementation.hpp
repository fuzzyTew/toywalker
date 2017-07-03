#pragma once

class ServoImplementation
{
public:
	virtual int id() const = 0;
	virtual float goRatio(float ratio) = 0;
	virtual float radiansPerRatio() const = 0;
	float go(float radians)
	{
		float radsPerRat = radiansPerRatio();
		return goRatio(radians / radsPerRat) * radsPerRat;
	}
};
