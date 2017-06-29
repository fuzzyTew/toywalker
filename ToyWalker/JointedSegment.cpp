#include "JointedSegment.hpp"

#include "JointAttachment.hpp"

JointedSegment & JointedSegment::attached(size_t id)
{
	return attachment(id).other(this);
}

MatrixAffine JointedSegment::toAttached(size_t id, float radians)
{
	return attachment(id).toOther(this, radians); 
}
