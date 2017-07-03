#pragma once

#include "Joint.hpp"
#include "JointedSegment.hpp"
#include "MatrixAffine.hpp"
#include "Vector.hpp"

/**
 * Represents the connection between two segments by a servo.
 */
class JointAttachment
{
public:
	JointAttachment(Joint & joint, JointedSegment & A, Vector positionA, Vector axisA, JointedSegment & B, MatrixAffine transformAtoB, float radiansAtTransform)
	: _joint(joint), segments{&A, &B}, _positionA(positionA), _axisA(axisA), _AToB(transformAtoB), referenceRadians(radiansAtTransform)
	{
		A.joints.append(this);
		B.joints.append(this);
	}

	~JointAttachment()
	{
		size_t i;
		for (i = 0; i < A().attachments(); ++ i)
			if (&A().attachment(i) == this) {
				A().joints.erase(i);
				break;
			}
		for (i = 0; i < B().attachments(); ++ i)
			if (&B().attachment(i) == this) {
				B().joints.erase(i);
				break;
			}
	}

	Joint & joint() { return _joint; }

	JointedSegment & segment(bool B) { return *segments[B ? 1 : 0]; }

	JointedSegment & A() { return *segments[0]; }
	Vector positionA() { return _positionA; }
	Vector axisA() { return _axisA; }
	
	JointedSegment & B() { return *segments[1]; }
	Vector positionB() { return _AToB * _positionA; }
	Vector axisB() { return _AToB * _axisA; }

	JointedSegment & other(JointedSegment * self) { return self == &A() ? B() : A(); }

	MatrixAffine AToB(float radians)
	{
		return _AToB
		       * MatrixAffine(_positionA)
		       * MatrixAffine(_axisA, radians - referenceRadians)
		       * MatrixAffine(-_positionA);
	}
	MatrixAffine BToA(float radians) { return AToB(radians).inverse(); }
	MatrixAffine toOther(JointedSegment * self, float radians)
	{
		MatrixAffine ret = AToB(radians);
		if (self == &B())
			ret.invert();
		return ret;
	}

private:

	Joint & _joint;
	
	JointedSegment * segments[2];

	Vector _positionA;
	Vector _axisA;

	MatrixAffine _AToB;
	float referenceRadians;
};
