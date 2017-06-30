#pragma once

#include <math.h>

class Vector
{
public:
	Vector()
	{ }

	Vector(float x, float y, float z)
	: x(x), y(y), z(z)
	{ }

	Vector operator-() const
	{
		return Vector(-x, -y, -z);
	}
	
	Vector operator*(float f) const
	{
		Vector r(*this);

		r *= f;

		return r;
	}

	Vector & operator*=(float f)
	{
		x *= f;
		y *= f;
		z *= f;

		return *this;
	}
	
	Vector operator/(float f) const
	{
		Vector r(*this);

		r /= f;

		return r;
	}

	Vector & operator/=(float f)
	{
		x /= f;
		y /= f;
		z /= f;

		return *this;
	}

	Vector operator+(const Vector & v) const
	{
		Vector r(*this);

		r += v;

		return r;
	}

	Vector & operator+=(const Vector & v)
	{
		x += v.x;
		y += v.y;
		z += v.z;

		return *this;
	}

	Vector operator-(const Vector & v) const
	{
		Vector r(*this);

		r -= v;

		return r;
	}

	Vector & operator-=(const Vector & v)
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;

		return *this;
	}

	float lengthSquared() const
	{
		return x * x + y * y + z * z;
	}

	float length() const
	{
		return sqrt(lengthSquared());
	}

	Vector normalized() const
	{
		Vector r = *this;

		r.normalize();

		return r;
	}

	void normalize()
	{
		(*this) /= length();
	}

	float x, y, z;
};
