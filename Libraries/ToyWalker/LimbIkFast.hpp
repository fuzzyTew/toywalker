#pragma once

#include "ToyWalker.h"

#include "IkFastSolutionArray.hpp"
#include "IKSolutionSelector.hpp"
#include "Limb.hpp"
#include "Servo.hpp"
#include "Vector.hpp"

namespace toywalker {

class LimbIkFast : public Limb
{
public:
	typedef bool (&ComputeIk)(const Real*, const Real*, const Real*, ikfast::IkSolutionListBase<Real>&);
	typedef void (&ComputeFk)(const Real*, Real*, Real*);

	template <typename... _Servos>
	LimbIkFast(Body & body,
	           Vector3 const & home,
	           Array2 reach,
	           ComputeIk computeIk_translation3d,
	           ComputeFk computeFk_translation3d,
		   IKSolutionSelector & selector,
	           _Servos &... servos)
	: Limb(body, home, reach, Servos::Map(std::initializer_list<Servo *>({&servos...}).begin(),sizeof...(servos))),
	  computeIkTranslation3D(computeIk_translation3d),
	  computeFkTranslation3D(computeFk_translation3d),
	  selector(selector)
	{
		activate();
	}

	virtual Vector3 footBody(Angles const & plan);

	virtual Angles planBody(Vector3 const & footDestination);

private:
	ComputeIk computeIkTranslation3D;
	ComputeFk computeFkTranslation3D;
	IKSolutionSelector & selector;
};

}
