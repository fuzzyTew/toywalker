#pragma once

#include "ToyWalker.h"

#include <Eigen/Core>

namespace toywalker {

	using Vector2 = Eigen::Matrix<Real, 2, 1>;
	using Vector3 = Eigen::Matrix<Real, 3, 1>;

	using Array2 = Eigen::Array<Real, 2, 1>;
	using Array3 = Eigen::Array<Real, 3, 1>;
	template <size_t X, typename Type = Real>
	using ArrayX = Eigen::Array<Type, Eigen::Dynamic, 1, 0, X, 1>;

	using Matrix3 = Eigen::Matrix<Real, 3, 3>;

}
