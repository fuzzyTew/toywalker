#pragma once

#include "Vector.hpp"

#include <math.h>

class MatrixAffine
{
public:
	MatrixAffine()
	{
		set(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0
		);
	}

	MatrixAffine(const Vector & translation, const Vector & axis, const float & radians)
	{
		setTranslation(translation);
		setAxisAngle(axis, radians);
	}

	float & operator()(int a, int b)
	{
		return m[a][b];
	}

	MatrixAffine operator*(const MatrixAffine & m2) const
	{
		MatrixAffine r = *this;

		r *= m2;

		return r;
	}

	MatrixAffine & operator*=(const MatrixAffine & m2)
	{
		set(
			m[0][0] * m2.m[0][0] + m[0][1] * m2.m[1][0] + m[0][2] * m2.m[2][0],
			m[0][0] * m2.m[0][1] + m[0][1] * m2.m[1][1] + m[0][2] * m2.m[2][1],
			m[0][0] * m2.m[0][2] + m[0][1] * m2.m[1][2] + m[0][2] * m2.m[2][2],
			m[0][0] * m2.m[0][3] + m[0][1] * m2.m[1][3] + m[0][2] * m2.m[2][3] + m[0][3],
			
			m[1][0] * m2.m[0][0] + m[1][1] * m2.m[1][0] + m[1][2] * m2.m[2][0],
			m[1][0] * m2.m[0][1] + m[1][1] * m2.m[1][1] + m[1][2] * m2.m[2][1],
			m[1][0] * m2.m[0][2] + m[1][1] * m2.m[1][2] + m[1][2] * m2.m[2][2],
			m[1][0] * m2.m[0][3] + m[1][1] * m2.m[1][3] + m[1][2] * m2.m[2][3] + m[1][3],
			
			m[2][0] * m2.m[0][0] + m[2][1] * m2.m[1][0] + m[2][2] * m2.m[2][0],
			m[2][0] * m2.m[0][1] + m[2][1] * m2.m[1][1] + m[2][2] * m2.m[2][1],
			m[2][0] * m2.m[0][2] + m[2][1] * m2.m[1][2] + m[2][2] * m2.m[2][2],
			m[2][0] * m2.m[0][3] + m[2][1] * m2.m[1][3] + m[2][2] * m2.m[2][3] + m[2][3]
		);
		return *this;
	}

	Vector operator*(const Vector & v) const
	{
		return Vector(
			m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3],
			m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3],
			m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]
		);
	}

	Vector transformedDirection(const Vector & v) const
	{
		return Vector(
			m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
			m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
			m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
		);
	}

	MatrixAffine inverse() const
	{
		MatrixAffine r = *this;

		r.invert();

		return r;
	}

	MatrixAffine & invert()
	{
		float m10 = m[1][0], m11 = m[1][1], m12 = m[1][2];
		float m20 = m[2][0], m21 = m[2][1], m22 = m[2][2];
		
		float t00 = m22 * m11 - m21 * m12;
		float t10 = m20 * m12 - m22 * m10;
		float t20 = m21 * m10 - m20 * m11;
		
		float m00 = m[0][0], m01 = m[0][1], m02 = m[0][2];
		
		float invDet = 1 / (m00 * t00 + m01 * t10 + m02 * t20);
		
		t00 *= invDet; t10 *= invDet; t20 *= invDet;
		
		m00 *= invDet; m01 *= invDet; m02 *= invDet;
		
		float r00 = t00;
		float r01 = m02 * m21 - m01 * m22;
		float r02 = m01 * m12 - m02 * m11;
		
		float r10 = t10;
		float r11 = m00 * m22 - m02 * m20;
		float r12 = m02 * m10 - m00 * m12;
		
		float r20 = t20;
		float r21 = m01 * m20 - m00 * m21;
		float r22 = m00 * m11 - m01 * m10;
		
		float m03 = m[0][3], m13 = m[1][3], m23 = m[2][3];
		
		float r03 = - (r00 * m03 + r01 * m13 + r02 * m23);
		float r13 = - (r10 * m03 + r11 * m13 + r12 * m23);
		float r23 = - (r20 * m03 + r21 * m13 + r22 * m23);
		
		set(
			r00, r01, r02, r03,
			r10, r11, r12, r13,
			r20, r21, r22, r23
		);

		return *this;
	}

	Vector getTranslation() const
	{
		return Vector(m[0][3], m[1][3], m[2][3]);
	}

	const Vector & setTranslation(const Vector & v)
	{
		m[0][3] = v.x;
		m[1][3] = v.y;
		m[2][3] = v.z;
		return v;
	}

	MatrixAffine(const Vector & v)
	{
		set(
			1, 0, 0, v.x,
			0, 1, 0, v.y,
			0, 0, 1, v.z
		);
	}

	MatrixAffine(float x, float y, float z)
	{
		set(
			1, 0, 0, x,
			0, 1, 0, y,
			0, 0, 1, z
		);
	}

	void getAxisAngle(Vector & axis, float & radians) const
	{
		float trace = m[0][0] + m[1][1] + m[2][2];
		float c = 0.5 * (trace - 1.0);
		radians = acos(c);  // in [0,PI]
		
		if (radians > 0.0)
		{
			if (radians < M_PI)
			{
				axis.x = m[2][1] - m[1][2];
				axis.y = m[0][2] - m[2][0];
				axis.z = m[1][0] - m[0][1];
				axis.normalize();
			}
			else
			{
				// angle is PI
				float halfInverse;
				if (m[0][0] >= m[1][1])
				{
					// r00 >= r11
					if (m[0][0] >= m[2][2])
					{
						// r00 is maximum diagonal term
						axis.x = 0.5 * sqrt(m[0][0] - m[1][1] - m[2][2] + 1.0);
						halfInverse = 0.5 / axis.x;
						axis.y = halfInverse * m[0][1];
						axis.z = halfInverse * m[0][2];
					}
					else
					{
						// r22 is maximum diagonal term
						axis.z = 0.5 * sqrt(m[2][2] - m[0][0] - m[1][1] + 1.0);
						halfInverse = 0.5 / axis.z;
						axis.x = halfInverse * m[0][2];
						axis.y = halfInverse * m[1][2];
					}
				}
				else
				{
					// r11 > r00
					if (m[1][1] >= m[2][2])
					{
						// r11 is maximum diagonal term
						axis.y = 0.5 * sqrt(m[1][1] - m[0][0] - m[2][2] + 1.0);
						halfInverse  = 0.5 / axis.y;
						axis.x = halfInverse * m[0][1];
						axis.z = halfInverse * m[1][2];
					}
					else
					{
						// r22 is maximum diagonal term
						axis.z = 0.5 * sqrt(m[2][2] - m[0][0] - m[1][1] + 1.0);
						halfInverse = 0.5 / axis.z;
						axis.x = halfInverse * m[0][2];
						axis.y = halfInverse * m[1][2];
					}
				}
			}
		}
		else
		{
			// The angle is 0 and the matrix is the identity.  Any axis will
			// work, so just use the x-axis.
			axis.x = 1.0;
			axis.y = 0.0;
			axis.z = 0.0;
		}
	}

	void setAxisAngle(const Vector & axis, const float & radians)
	{
		Vector nAxis = axis.normalized();

		float c = cos(radians);
		float s = sin(radians);
		float oneMinusCos = 1.0 - c;
		float x2 = nAxis.x * nAxis.x;
		float y2 = nAxis.y * nAxis.y;
		float z2 = nAxis.z * nAxis.z;
		float xym = nAxis.x * nAxis.y * oneMinusCos;
		float xzm = nAxis.x * nAxis.z * oneMinusCos;
		float yzm = nAxis.y * nAxis.z * oneMinusCos;
		float xs = nAxis.x * s;
		float ys = nAxis.y * s;
		float zs = nAxis.z * s;

		m[0][0] = x2 * oneMinusCos + c;
		m[0][1] = xym - zs;
		m[0][2] = xzm + ys;
		m[1][0] = xym + zs;
		m[1][1] = y2 * oneMinusCos + c;
		m[1][2] = yzm - xs;
		m[2][0] = xzm - ys;
		m[2][1] = yzm + xs;
		m[2][2] = z2 * oneMinusCos + c;
	}

	MatrixAffine(const Vector & axis, const float & radians)
	{
		setAxisAngle(axis, radians);
		setTranslation(Vector(0, 0, 0));
	}

private:
	float m[3][4];

	MatrixAffine(float m00, float m01, float m02, float m03,
	             float m10, float m11, float m12, float m13,
	             float m20, float m21, float m22, float m23)
	{
		m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
		m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
		m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
	}

	void set(float m00, float m01, float m02, float m03,
	         float m10, float m11, float m12, float m13,
	         float m20, float m21, float m22, float m23)
	{
		m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
		m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
		m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
	}
};
