#pragma once

#include "ArrayDynamic.hpp"
#include "MatrixAffine.hpp"

class JointAttachment;

class JointedSegment
{
friend class JointAttachment;
public:
	JointedSegment() {}

	size_t attachments() { return joints.size(); }

	JointAttachment & attachment(size_t id) { return *joints[id]; }
	
	JointedSegment & attached(size_t id);

	MatrixAffine toAttached(size_t id, float radians);

private:
	ArrayDynamic<JointAttachment*> joints;
};
