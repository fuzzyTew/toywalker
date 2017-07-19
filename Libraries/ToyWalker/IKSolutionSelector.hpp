#pragma once

#include "Limb.hpp"

namespace toywalker {

class IKSolutionSelector
{
public:
	virtual size_t select(Limb & limb, Limb::Angles * solutions, size_t nSolutions) = 0;
};

// prioritizes solutions that have similar angles among the chosen joints, multiplied by the given multipliers
class IKSolutionNatural : public IKSolutionSelector
{
public:
	struct Joint
	{
		size_t index;
		double multiplier;
		Joint(size_t index = 0, double multiplier = 1)
		: index(index), multiplier(multiplier)
		{ }
	};

	typedef Eigen::Array<Joint, Eigen::Dynamic, 1, 0, Limb::MAX_JOINTS, 1> Joints;

	IKSolutionNatural(std::initializer_list<Joint> joints)
	: joints(Joints::Map(joints.begin(), joints.size()))
	{ }

	size_t select(Limb & limb, Limb::Angles * solutions, size_t nSolutions)
	{
		double minSpread = INFINITY;
		size_t minIdx = 0;

		for (size_t i = 0; i < nSolutions; ++ i)
		{
			double avg = 0;
			size_t j;
			for (j = 0; j < joints.size(); ++ j)
				avg += jointValue(solutions[i], j);
			avg /= solutions[i].size();
			double spread = 0;
			for (j = 0; j < joints.size(); ++ j)
				spread += abs(jointValue(solutions[i], j) - avg);
			if (spread < minSpread) {
				minSpread = spread;
				minIdx = i;
			}
		}

		return minIdx;
	}

	const Joints joints;

private:
	double jointValue(Limb::Angles & solution, size_t joint)
	{
		return solution[joints[joint].index] * joints[joint].multiplier;
	}
};

}
