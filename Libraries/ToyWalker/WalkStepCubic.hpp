#pragma once

#include "ToyWalker.h"

#include "WalkStep.hpp"

#include "Limb.hpp"

namespace toywalker {

// Uses a cubic curve in area space to ensure the feet strike the ground precisely and gently.
class WalkStepCubic : public WalkStep
{
public:
	WalkStepCubic(Walker & walker, Limb & limb, Real stepHeight);

	Real stepHeight() { return _stepHeight; }
	void stepHeight(Real height) { _stepHeight = height; }

	virtual void start();

	virtual Limb::Angles at(Real seconds);

private:
	Real _stepHeight;
	Eigen::Array<Real, 3, 4> coeffs;

	void calculateCoefficients();
};

}

// impl

namespace toywalker {

WalkStepCubic::WalkStepCubic(Walker & walker, Limb & limb, Real stepHeight)
: WalkStep(walker, limb),
  _stepHeight(stepHeight)
{ }

void WalkStepCubic::start()
{
	Vector3 const & origin = originArea();
	Vector3 const & dest = destinationArea();
	Real dur = duration();
	Real peak = dur / 3;
	//
	// pos(t) = at^3 + bt^2 + ct + d
	// pos(0) = origin
	// pos(duration) = dest
	// pos'(t) = 3at^2 + 2bt + c
	// pos'(0).x = 0
	// pos'(0).y = 0
	// pos'(duration) = <0,0,0>
	//
	// I tried a test graph with duration=1, and the maximum is at time = 1/3 .
	//
	//                   [0^3   3*0^2   dur^3 3dur^2]
	// [a.x b.x c.x d.x] [0^2    2*0    dur^2  2dur ] = [origin.x      0     dest.x 0]
	// [a.y b.y c.y d.y] [ 0      1      dur     1  ]   [origin.y      0     dest.y 0]
	//                   [ 1      0       1      0  ]
	//
	//                   [0^3 (dur/3)^3 dur^3 3dur^2]
	// [a.z b.z c.z d.z] [0^2 (dur/3)^2 dur^2  2dur ] = [origin.z stepHeight dest.z 0]
	//                   [ 0   (dur/3)   dur     1  ]
	//                   [ 1      1       1      0  ]


	coeffs.topRows<2>() =
		(Eigen::Matrix<Real,2,4>() <<
			origin.x(), 0, dest.x(), 0,
			origin.y(), 0, dest.y(), 0
		).finished() * (Eigen::Matrix<Real,4,4>() <<
			0*0*0, 3*0*0, dur*dur*dur, 3*dur*dur,
			 0*0,   2*0,    dur*dur,     2*dur,
			  0,     1,       dur,         1,
			  1,     0,        1,          0
		).finished().inverse();
	coeffs.bottomRows<1>() =
		(Eigen::Matrix<Real,1,4>() <<
		 	origin.z(), stepHeight(), dest.z(), 0
		).finished() * (Eigen::Matrix<Real,4,4>() <<
			0*0*0, peak*peak*peak, dur*dur*dur, 3*dur*dur,
			 0*0,     peak*peak,     dur*dur,     2*dur,
			  0,         peak,         dur,         1,
			  1,          1,            1,          0
		).finished().inverse();
}

Limb::Angles WalkStepCubic::at(Real seconds)
{
	Vector3 area = ((coeffs.col(0) * seconds + coeffs.col(1)) * seconds + coeffs.col(2)) * seconds + coeffs.col(3);
	return limb().planArea(area);
}

}
