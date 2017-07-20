#pragma once

#include "LimbIkFast.hpp"

#include <string>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace toywalker {

class LimbIkFastKDL : public LimbIkFast
{
public:
	template <typename... _Servos>
	LimbIkFastKDL(KDL::Tree & kdl, std::string bodySegment, std::string footSegment, ComputeIk computeIk_translation3d, ComputeFk computeFk_translation3d, _Servos &... servos)
	: LimbIkFast(
		(kdl.getChain(bodySegment, footSegment, chain),
		 Vector3::Map(chain.getSegment(1).getJoint().JointOrigin().data, 3)),
		computeIk_translation3d,
		computeFk_translation3d,
		servos...
	)
	{ }

private:
	KDL::Chain chain;
};

}
