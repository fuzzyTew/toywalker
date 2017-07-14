#pragma once

#include <cmath>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "ikfast.h"

#include "IkFastSolutionArray.hpp"

#include "Limb.hpp"

namespace toywalker {

template <
	bool (&ComputeIk_translation3d)(const double*, const double*, const double*, ikfast::IkSolutionListBase<double>&),
	void (&ComputeFk_translation3d)(const double*, double*, double*),
	typename... Servos
>
class LimbIkFast : public Limb
{
public:
	typedef Eigen::Matrix<double, sizeof...(Servos), 1> Angles;

	std::tuple<Servos...> servos;

	LimbIkFast(Eigen::Vector3d const & hipPos)
	: Limb(hipPos, sizeof...(Servos))
	{
		initImpl<Servos...>();
	}

	LimIkFast(Servos... servos, Eigen::Vector3d const & hipPos)
	: Limb(hipPos, sizeof...(Servos))
	  servos{servos...}
	{
		initImpl<Servos...>();
	}

	void go(Eigen::Vector3d const & pos)
	{
		if (computeIk(pos, angles))
			goImpl<Servos...>(angles);
	}

	Limb::Angles plan(Eigen::Vector3d const & pos)
	{
		Angles ret;
		computeIk(pos, ret);
		return ret;
	}

	void execute(Limb::Angles const & setup)
	{
		goImpl<Servos...>(setup);
	}

	double go(unsigned int servo, double radians)
	{
		return goServoImpl<Servos...>(servo, radians);
	}

	Eigen::Vector3d foot()
	{
		Eigen::Vector3d ret;
		ComputeFk_translation3d(&angles(0), &ret(0), nullptr);
		return ret;
	}

	Eigen::Vector3d foot(Limb::Angles const & setup)
	{
		Eigen::Vector3d ret;
		ComputeFk_translation3d(&setup(0), &ret(0), nullptr);
		return ret;
	}

	Limb::Angles servoRadians()
	{
		return angles;
	}

	double servoRadians(unsigned int servo)
	{
		return angles[servo];
	}

private:
	Angles angles;

	bool computeIk(Eigen::Vector3d const & pos, Angles & solution)
	{
		IkFastSolutionArray<sizeof...(Servos)> & solutions = IkFastSolutionArray<sizeof...(Servos)>::instance;

		solutions.Clear();

		bool success = ComputeIk_translation3d(&pos[0], nullptr, nullptr, solutions);
		if (success) {
			double bestDistance = INFINITY;
			const ikfast::IkSolutionBase<double> * bestSolution = nullptr;
	
			for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++ i)
			{
				double distanceSum = 0;
				const ikfast::IkSolutionBase<double> & curSolution = solutions.GetSolution(i);
				Angles curValues;
				
				curSolution.GetSolution(&curValues(0), NULL);
				distanceSum = distImpl<Servos...>(curValues);
	
				if (distanceSum < bestDistance) {
					bestDistance = distanceSum;
					bestSolution = &curSolution;
				}
	
			}
	
			if (bestSolution != nullptr)
				bestSolution->GetSolution(&solution(0), NULL);
			else
				return false;
		}
		return success;
	}

	template <int basecase = 0>
	inline void goImpl(const Angles & angles)
	{ }

	template <typename Servo, typename... Remaining>
	inline void goImpl(const Angles & angles)
	{
		constexpr int i = sizeof...(Servos) - sizeof...(Remaining) - 1;
		std::get<i>(servos).go(angles(i));
		this->angles[i] = std::get<i>(servos).where();
		goImpl<Remaining...>(angles);
	}

	template <int basecase = 0>
	inline double distImpl(const Angles & angles)
	{
		return 0;
	}

	template <typename Servo, typename... Remaining>
	inline double distImpl(const Angles & angles)
	{
		constexpr int i = sizeof...(Servos) - sizeof...(Remaining) - 1;
		if (angles(i) < std::get<i>(servos).softmin() || angles(i) > std::get<i>(servos).softmax())
			return INFINITY;
		return std::abs(this->angles(i) - angles(i)) + distImpl<Remaining...>(angles);
	}

	template <int basecase = 0>
	inline double goServoImpl(unsigned int servo, double radians)
	{
		return 0;
	}

	template <typename Servo, typename... Remaining>
	inline double goServoImpl(unsigned int servo, double radians)
	{
		constexpr int i = sizeof...(Servos) - sizeof...(Remaining) - 1;
		if (servo == i) {
			return angles(i) = std::get<i>(servos).go(radians);
		} else {
			return goServoImpl<Remaining...>(servo, radians);
		}
	}

	template <int basecase = 0>
	inline void initImpl()
	{ }

	template <typename Servo, typename... Remaining>
	inline void initImpl()
	{
		constexpr int i = sizeof...(Servos) - sizeof...(Remaining) - 1;
		angles(i) = std::get<i>(servos).where();
		initImpl<Remaining...>();
	}
};

}
