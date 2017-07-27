#pragma once

#include "Limb.hpp"

namespace toywalker {

class IKSolutionSelector
{
public:
	virtual size_t select(Limb & limb, Limb::Angles * solutions, size_t nSolutions) = 0;
};

// prioritizes solutions that have similar angles among the chosen joints, multiplied by the given multipliers
// if multiple solutions all have similar angles, chooses the one nearest to the current pose
class IKSolutionNatural : public IKSolutionSelector
{
public:
	struct Joint
	{
		size_t index;
		Real multiplier;
		Joint(size_t index = 0, Real multiplier = 1)
		: index(index), multiplier(multiplier)
		{ }
	};

	typedef Eigen::Array<Joint, Eigen::Dynamic, 1, 0, Limb::MAX_JOINTS, 1> Joints;

	IKSolutionNatural(std::initializer_list<Joint> joints, Real epsilon = 0.125)
	: joints(Joints::Map(joints.begin(), joints.size())), epsilon(epsilon)
	{ }

	size_t select(Limb & limb, Limb::Angles * solutions, size_t nSolutions)
	{
		auto pose = limb.anglesGoal();
		Real minDistance = INFINITY;
		Real minSpread = INFINITY;
		size_t minIdx = 0;

		for (size_t i = 0; i < nSolutions; ++ i)
		{
			Real avg = 0;
			int j;
			for (j = 0; j < joints.size(); ++ j)
				avg += jointValue(solutions[i], j);
			avg /= solutions[i].size();
			Real spread = 0;
			for (j = 0; j < joints.size(); ++ j)
				spread += abs(jointValue(solutions[i], j) - avg);
			if (spread + epsilon < minSpread) {
				minSpread = spread;
				minIdx = i;
				minDistance = 0;
				for (j = 0; j < solutions[i].size(); ++ j) {
					minDistance += abs(solutions[i][j] - pose[j]);
				}
			} else if (abs(spread - minSpread) <= epsilon) {
				Real distance = 0;
				for (j = 0; j < solutions[i].size(); ++ j) {
					distance += abs(solutions[i][j] - pose[j]);
				}
				if (distance < minDistance) {
					minDistance = distance;
					minSpread = spread;
					minIdx = i;
				}
			}
		}

		return minIdx;
	}

	const Joints joints;
	const Real epsilon;

private:
	Real jointValue(Limb::Angles & solution, size_t joint)
	{
		return solution[joints[joint].index] * joints[joint].multiplier;
	}
};

// Picks the solution closest to the previous pose
class IKSolutionNearest : public IKSolutionSelector
{
public:
	size_t select(Limb & limb, Limb::Angles * solutions, size_t nSolutions)
	{
		auto pose = limb.anglesGoal();

		Real minDistance = INFINITY;
		size_t minIdx = 0;

		for (size_t i = 0; i < nSolutions; ++ i)
		{
			Real distance = 0;
			for (int j = 0; j < solutions[i].size(); ++ j) {
				distance += abs(solutions[i][j] - pose[j]);
			}
			if (distance < minDistance) {
				minDistance = distance;
				minIdx = i;
			}
		}

		return minIdx;
	}
};

}
