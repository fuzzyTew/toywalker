#pragma once

#include "ToyWalker.h"

#include <cmath>
#include <tuple>
#include <vector>

#include <Eigen/Core>

#include "ikfast.h"

#include "IkFastSolutionArray.hpp"

#include "IKSolutionSelector.hpp"
#include "Limb.hpp"
#include "Servo.hpp"

namespace toywalker {

class LimbIkFast : public Limb
{
public:
	typedef bool (&ComputeIk)(const double*, const double*, const double*, ikfast::IkSolutionListBase<double>&);
	typedef void (&ComputeFk)(const double*, double*, double*);

	template <typename... _Servos>
	LimbIkFast(Eigen::Vector3d const & home,
	           Eigen::Array2d reach,
	           ComputeIk computeIk_translation3d,
	           ComputeFk computeFk_translation3d,
		   IKSolutionSelector & selector,
	           _Servos &... servos)
	: Limb(home, reach, Servos::Map(std::initializer_list<Servo *>({&servos...}).begin(),sizeof...(servos))),
	  computeIkTranslation3D(computeIk_translation3d),
	  computeFkTranslation3D(computeFk_translation3d),
	  selector(selector)
	{
		activate();
	}

	virtual Eigen::Vector3d foot(Angles const & plan);

	virtual Angles plan(Eigen::Vector3d const & footDestination);

private:
	ComputeIk computeIkTranslation3D;
	ComputeFk computeFkTranslation3D;
	IKSolutionSelector & selector;
};

}
