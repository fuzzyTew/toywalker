#pragma once

#include "ToyWalker.h"

#include <Eigen/Geometry>

namespace toywalker {

	// an isometry is a parallel-preserving transform which is known to have no scaling or shear
	using Isometry2 = Eigen::Transform<Real, 2, Eigen::Isometry>;
	using Isometry3 = Eigen::Transform<Real, 3, Eigen::Isometry>;

	// an affine transform is anything parallel-preserving
	using Affine2 = Eigen::Transform<Real, 2, Eigen::Affine>;
	using Affine3 = Eigen::Transform<Real, 3, Eigen::Affine>;

	using Plane = Eigen::Hyperplane<Real, 3>;

	using Translation2 = Eigen::Translation<Real, 2>;
	using Translation3 = Eigen::Translation<Real, 3>;

	using AngleAxis = Eigen::AngleAxis<Real>;
}
