#pragma once

class ServoImplementation
{
public:
	virtual int id() const = 0;
	virtual double go(double radians) = 0;
};
