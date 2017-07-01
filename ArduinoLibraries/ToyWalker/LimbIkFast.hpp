#pragma once

#include <cmath>
#include <vector>

#include <boost/tr1/tuple.hpp>
#include <Eigen/Core>

#include "ikfast.h"

#include "IkFastSolutionArray.hpp"

template <
	bool (&ComputeIk_translation3d)(const double*, const double*, const double*, ikfast::IkSolutionListBase<double>&),
	void (&ComputeFk_translation3d)(const double*, double*, double*),
	typename... Servos
>
class LimbIkFast
{
public:
	typedef Eigen::Matrix<double, sizeof...(Servos), 1> Angles;

	void go(Eigen::Vector3d const & pos)
	{
		if (computeIk(pos, angles)) {
			goImpl<Servos...>(angles);
		}
			
	}

	Angles plan(Eigen::Vector3d const & pos)
	{
		Angles ret;
		computeIk(pos, ret);
		return ret;
	}

	void goPlan(Angles const & setup)
	{
		goImpl<Servos...>(setup);
	}

	Eigen::Vector3d where()
	{
		Eigen::Vector3d ret;
		ComputeFk_translation3d(&angles(0), &ret(0), nullptr);
		return ret;
	}

	Eigen::Vector3d wherePlan(Angles const & setup)
	{
		Eigen::Vector3d ret;
		ComputeFk_translation3d(&setup(0), &ret(0), nullptr);
		return ret;
	}

	std::tr1::tuple<Servos...> servos;

private:
	Angles angles;

	bool computeIk(Eigen::Vector3d const & pos, Angles & solution)
	{
		IkFastSolutionArray<sizeof...(Servos)> & solutions = IkFastSolutionArray<sizeof...(Servos)>::instance;

		solutions.Clear();

		bool success = ComputeIk_translation3d(&pos[0], nullptr, nullptr, solutions);
		if (success) {
			double bestDistance = INFINITY;
			const ikfast::IkSolutionBase<double> * bestSolution;
	
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
	
			bestSolution->GetSolution(&solution(0), NULL);
		}
		return success;
	}

	template <int basecase = 0>
	void goImpl(const Angles & angles)
	{ }

	template <typename Servo, typename... Remaining>
	void goImpl(const Angles & angles)
	{
		constexpr int i = sizeof...(Servos) - sizeof...(Remaining) - 1;
		std::tr1::get<i>(servos).go(angles(i));
		this->angles[i] = std::tr1::get<i>(servos).where();
		goImpl<Remaining...>(angles);
	}

	template <int basecase = 0>
	double distImpl(const Angles & angles)
	{
		return 0;
	}

	template <typename Servo, typename... Remaining>
	double distImpl(const Angles & angles)
	{
		constexpr int i = sizeof...(Servos) - sizeof...(Remaining) - 1;
		//Serial.println(angles(i));
		return std::abs(std::tr1::get<i>(servos).where() - angles(i)) + distImpl<Remaining...>(angles);
	}
};

